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
 * Control modes mirror the firmware: manual, pwm, preheat, pause, baking.
 */

const TICK_MS = 500; // matches WS_INTERVAL on the firmware
const ROOM_TEMP = 22; // °C ambient
const MAX_HEAT_RATE = 14; // °C/s at peak of S-curve
const NOISE_AMP = 0.35; // °C peak random noise

// PI-mode parameters – keep in sync with the firmware (main.cpp)
const PAUSE_TEMP = 275; // low keep-warm hold
const BAKE_BOOST = 40; // °C added on top of target while baking
const BAKE_WAIT_MS = 60 * 1000; // 1 min warm-up before baking
const BAKE_DURATION_MS = 3 * 60 * 1000; // 3 min bake, then back to preheat

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
	pause_temp: number;
	bake_boost: number;
	pwm_on: number;
	pwm_off: number;
	pid: number;
	kp: number;
	ki: number;
	kd: number;
	bake_phase: string;
	bake_remaining: number;
};

type Command = { cmd: string; value?: number | string };

export class SpoofSocket {
	onopen: (() => void) | null = null;
	onclose: (() => void) | null = null;
	onmessage: ((e: { data: string }) => void) | null = null;
	readyState: number = WebSocket.CONNECTING;

	private s: SensorPayload = {
		mode: 'preheat',
		temperature: ROOM_TEMP,
		relais: 0,
		target_temp: 460,
		pause_temp: PAUSE_TEMP,
		bake_boost: BAKE_BOOST,
		pwm_on: 2000,
		pwm_off: 4000,
		pid: 0,
		kp: 0.6,
		ki: 0.1,
		kd: 0.0,
		bake_phase: '',
		bake_remaining: 0
	};

	private integral = 0;
	private lastSwitch = Date.now();
	private bakeStart = 0;
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

	private isPidMode(m: string): boolean {
		return m === 'preheat' || m === 'pause' || m === 'baking';
	}

	/** Effective setpoint the PI loop drives towards for the current mode. */
	private activeSetpoint(): number {
		const s = this.s;
		if (s.mode === 'pause') return s.pause_temp;
		if (s.mode === 'baking') return s.target_temp + s.bake_boost;
		return s.target_temp;
	}

	private applyMode(m: string): void {
		const s = this.s;
		s.mode = m;
		if (m === 'baking') this.bakeStart = Date.now();
		if (m === 'manual') {
			s.relais = 0;
			this.lastSwitch = Date.now();
		}
		// Dump windup when the new target is below the current temp (e.g. -> pause)
		if (this.isPidMode(m) && this.activeSetpoint() < s.temperature) this.integral = 0;
	}

	private handle(cmd: string, value?: number | string): void {
		const s = this.s;
		switch (cmd) {
			case 'getReadings':
				this.emit();
				break;
			case 'setTargetTemp':
				s.target_temp = Math.max(0, Math.min(600, Number(value)));
				break;
			case 'setPauseTemp':
				s.pause_temp = Math.max(0, Math.min(600, Number(value)));
				break;
			case 'setBakeOffset':
				s.bake_boost = Math.max(0, Math.min(150, Number(value)));
				break;
			case 'setMode':
				this.applyMode(String(value));
				break;
			case 'switchRelais':
				// Manual override: take manual control and toggle the relay.
				s.mode = 'manual';
				s.relais = s.relais ? 0 : 1;
				this.lastSwitch = Date.now();
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
		} else if (this.isPidMode(s.mode)) {
			// Baking = warm-up + bake; when it's all done, return to preheat.
			if (s.mode === 'baking' && now - this.bakeStart >= BAKE_WAIT_MS + BAKE_DURATION_MS) {
				this.applyMode('preheat');
			}

			const error = this.activeSetpoint() - s.temperature;
			const pTerm = s.kp * error;
			let output = pTerm + this.integral;

			// Conditional-integration anti-windup (mirrors PIDController::compute)
			const satHigh = output >= 100 && error > 0;
			const satLow = output <= 0 && error < 0;
			if (!satHigh && !satLow) {
				this.integral = Math.max(-100, Math.min(100, this.integral + s.ki * error * dt));
			}
			output = Math.max(0, Math.min(100, pTerm + this.integral));
			s.pid = output;

			if (output >= 100) {
				s.relais = 1;
			} else if (output <= 0) {
				s.relais = 0;
			} else {
				const offDelay = s.pwm_on / (output / 100) - s.pwm_on;
				const elapsed = now - this.lastSwitch;
				if (s.relais === 1 && elapsed > s.pwm_on) {
					s.relais = 0;
					this.lastSwitch = now;
				} else if (s.relais === 0 && elapsed > offDelay) {
					s.relais = 1;
					this.lastSwitch = now;
				}
			}
		}

		// ── thermal model ────────────────────────────────────────────────────
		if (s.relais) {
			s.temperature += MAX_HEAT_RATE * heatFactor(s.temperature, this.activeSetpoint()) * dt;
		} else {
			s.temperature -= coolingRate(s.temperature) * dt;
		}
		s.temperature = Math.max(ROOM_TEMP, s.temperature);
		s.temperature += (Math.random() - 0.5) * NOISE_AMP;

		this.emit();
	}

	private emit(): void {
		const s = this.s;
		if (s.mode === 'baking') {
			const elapsed = Date.now() - this.bakeStart;
			if (elapsed < BAKE_WAIT_MS) {
				s.bake_phase = 'wait';
				s.bake_remaining = Math.max(0, Math.ceil((BAKE_WAIT_MS - elapsed) / 1000));
			} else {
				s.bake_phase = 'bake';
				s.bake_remaining = Math.max(
					0,
					Math.ceil((BAKE_WAIT_MS + BAKE_DURATION_MS - elapsed) / 1000)
				);
			}
		} else {
			s.bake_phase = '';
			s.bake_remaining = 0;
		}
		this.onmessage?.({ data: JSON.stringify(s) });
	}
}
