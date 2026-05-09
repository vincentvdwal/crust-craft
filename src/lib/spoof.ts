/**
 * SpoofSocket – a fake WebSocket that simulates the ESP32 firmware.
 * Used in DEV mode only. Cast to WebSocket with `as unknown as WebSocket`.
 *
 * Thermal model:
 *   – S-curve heating: rate peaks at ~50% of the target range (fast in the
 *     middle, slow at start/end), matching how a real oven behaves.
 *   – Newton's law of cooling when relay is off.
 *   – Small Gaussian noise on each tick for realism.
 *
 * Control modes mirror the firmware: manual, pwm, pid.
 */

const TICK_MS = 500; // matches WS_INTERVAL on the firmware
const ROOM_TEMP = 22; // °C ambient
const MAX_HEAT_RATE = 14; // °C/s at peak of S-curve
const NOISE_AMP = 0.35; // °C peak random noise

/** S-curve heating factor: parabolic 4x(1-x), peaks at x = 0.5. */
function heatFactor(temp: number, target: number): number {
	if (target <= ROOM_TEMP) return 0;
	const x = Math.max(0, Math.min(1, (temp - ROOM_TEMP) / (target - ROOM_TEMP)));
	return 4 * x * (1 - x) + 0.02; // +0.02 so the oven starts moving immediately
}

/** Newton's law of cooling – rate proportional to excess over ambient. */
function coolingRate(temp: number): number {
	return (1.4 * Math.max(0, temp - ROOM_TEMP)) / 300;
}

type SensorPayload = {
	mode: string;
	temperature: number;
	relais: number;
	target_temp: number;
	pwm_on: number;
	pwm_off: number;
	pid: number;
	kp: number;
	ki: number;
	kd: number;
};

type Command = { cmd: string; value?: number | string };

export class SpoofSocket {
	onopen: (() => void) | null = null;
	onclose: (() => void) | null = null;
	onmessage: ((e: { data: string }) => void) | null = null;
	readyState: number = WebSocket.CONNECTING;

	private s: SensorPayload = {
		mode: 'manual',
		temperature: ROOM_TEMP,
		relais: 0,
		target_temp: 460,
		pwm_on: 2000,
		pwm_off: 4000,
		pid: 0,
		kp: 0.6,
		ki: 0.1,
		kd: 0.0
	};

	private integral = 0;
	private lastSwitch = Date.now();
	private ticker: ReturnType<typeof setInterval> | null = null;

	constructor() {
		// Simulate async connection handshake
		setTimeout(() => {
			this.readyState = WebSocket.OPEN;
			this.onopen?.();
			this.ticker = setInterval(() => this.tick(), TICK_MS);
		}, 80);
	}

	send(raw: string): void {
		try {
			const { cmd, value } = JSON.parse(raw) as Command;
			this.handle(cmd, value);
		} catch {
			// ignore malformed messages
		}
	}

	/** Stop the simulation ticker without triggering onclose (for cleanup). */
	destroy(): void {
		if (this.ticker !== null) {
			clearInterval(this.ticker);
			this.ticker = null;
		}
	}

	close(): void {
		this.destroy();
		this.readyState = WebSocket.CLOSED;
		this.onclose?.();
	}

	private handle(cmd: string, value?: number | string): void {
		const s = this.s;
		switch (cmd) {
			case 'getReadings':
				this.emit();
				break;
			case 'setTargetTemp':
				s.target_temp = Number(value);
				this.integral = 0; // reset integral on setpoint change
				break;
			case 'setMode':
				s.mode = String(value);
				if (s.mode === 'manual') s.relais = 0;
				this.integral = 0;
				break;
			case 'switchRelais':
				if (s.mode === 'manual') {
					s.relais = s.relais ? 0 : 1;
					this.lastSwitch = Date.now();
				}
				break;
			case 'setPWMOn':
				s.pwm_on = Number(value) * 1000;
				break;
			case 'setPWMOff':
				s.pwm_off = Number(value) * 1000;
				break;
			case 'setKp':
				s.kp = Number(value);
				break;
			case 'setKi':
				s.ki = Number(value);
				break;
			case 'setKd':
				s.kd = Number(value);
				break;
		}
	}

	private tick(): void {
		const dt = TICK_MS / 1000;
		const s = this.s;
		const now = Date.now();

		// ── relay control (mirrors firmware regulateRelais) ──────────────────
		if (s.mode === 'pwm') {
			const elapsed = now - this.lastSwitch;
			if (s.relais === 1 && elapsed > s.pwm_on) {
				s.relais = 0;
				this.lastSwitch = now;
			} else if (s.relais === 0 && elapsed > s.pwm_off) {
				s.relais = 1;
				this.lastSwitch = now;
			}
		} else if (s.mode === 'pid') {
			const error = s.target_temp - s.temperature;
			this.integral = Math.max(-100, Math.min(100, this.integral + s.ki * error * dt));
			const output = Math.max(0, Math.min(100, s.kp * error + this.integral));
			s.pid = output;

			if (output > 0) {
				const offDelay = s.pwm_on / (output / 100) - s.pwm_on;
				const elapsed = now - this.lastSwitch;
				if (s.relais === 1 && offDelay > 0 && elapsed > s.pwm_on) {
					s.relais = 0;
					this.lastSwitch = now;
				} else if (s.relais === 0 && offDelay > 0 && elapsed > offDelay) {
					s.relais = 1;
					this.lastSwitch = now;
				}
			} else {
				s.relais = 0;
			}
		}

		// ── thermal model ────────────────────────────────────────────────────
		if (s.relais) {
			s.temperature += MAX_HEAT_RATE * heatFactor(s.temperature, s.target_temp) * dt;
		} else {
			s.temperature -= coolingRate(s.temperature) * dt;
		}
		s.temperature = Math.max(ROOM_TEMP, s.temperature);
		s.temperature += (Math.random() - 0.5) * NOISE_AMP;

		this.emit();
	}

	private emit(): void {
		this.onmessage?.({ data: JSON.stringify(this.s) });
	}
}
