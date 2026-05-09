<script lang="ts">
	import { onMount } from 'svelte';
	import { page } from '$app/state';
	import { SpoofSocket } from '$lib/spoof';
	import { Sun, Moon } from 'lucide-svelte';
	import {
		Chart,
		LineController,
		LineElement,
		PointElement,
		LinearScale,
		TimeScale,
		Filler,
		Tooltip,
		type ChartDataset
	} from 'chart.js';
	import 'chartjs-adapter-date-fns';

	Chart.register(
		LineController,
		LineElement,
		PointElement,
		LinearScale,
		TimeScale,
		Filler,
		Tooltip
	);

	// Rolling window for the chart (~5 min at 2 pts/sec)
	const CHART_WINDOW = 600;
	// Full buffer kept for CSV export (24h at 2 pts/sec)
	const MAX_STORED = 24 * 2 * 60 * 60;

	let gateway = `ws://${page.url.host}/ws`;
	let ws: WebSocket;

	let mode = $state('manual');
	let temp = $state(0);
	let relais = $state(0);
	let targetTemp = $state(460);
	let calculatedPidOutput = $state(0);

	let pwmOn = $state(2);
	let pwmOff = $state(4);

	let kp = $state(0.6);
	let ki = $state(0.1);
	let kd = $state(0.0);

	let pauseGraphUpdate = $state(false);
	let darkMode = $state(true);

	$effect(() => {
		document.documentElement.setAttribute('data-theme', darkMode ? 'dark' : 'light');
		if (chart) {
			const tickColor = darkMode ? '#7a7f96' : '#6b7280';
			const gridColor = darkMode ? 'rgba(255,255,255,0.05)' : 'rgba(0,0,0,0.07)';
			(chart.options.scales!.x as any).ticks.color = tickColor;
			(chart.options.scales!.x as any).grid.color = gridColor;
			(chart.options.scales!.y as any).ticks.color = tickColor;
			(chart.options.scales!.y as any).grid.color = gridColor;
			chart.update('none');
		}
	});

	// Temperature colour / class helpers
	const TEMP_ZONE_COLORS = [
		{ max: 350, color: '#22c55e' },
		{ max: 420, color: '#eab308' },
		{ max: 480, color: '#f97316' },
		{ max: 520, color: '#ef4444' }
	] as const;

	function tempColor(t: number): string {
		for (const z of TEMP_ZONE_COLORS) if (t < z.max) return z.color;
		return '#ef4444';
	}
	function tempIsExtreme(t: number): boolean {
		return t >= 520;
	}

	// Custom Chart.js plugin that draws temperature zone bands behind the data
	const tempZonesPlugin = {
		id: 'tempZones',
		beforeDatasetsDraw(c: Chart) {
			const { ctx, scales } = c;
			const xs = scales['x'];
			const ys = scales['y'];
			if (!xs || !ys) return;

			const left = xs.left;
			const right = xs.right;
			const width = right - left;

			const zones = [
				{ min: 0, max: 350, fill: 'rgba(34,197,94,0.07)', stripe: false },
				{ min: 350, max: 420, fill: 'rgba(234,179,8,0.09)', stripe: false },
				{ min: 420, max: 480, fill: 'rgba(249,115,22,0.10)', stripe: false },
				{ min: 480, max: 520, fill: 'rgba(239,68,68,0.12)', stripe: false },
				{ min: 520, max: 700, fill: 'rgba(239,68,68,0.06)', stripe: true }
			];
			const labels = ['Low', 'Medium', 'High', 'Very High', 'Extreme'];

			ctx.save();
			ctx.beginPath();
			ctx.rect(left, ys.top, width, ys.bottom - ys.top);
			ctx.clip();

			for (let i = 0; i < zones.length; i++) {
				const z = zones[i];
				const yTop = ys.getPixelForValue(z.max);
				const yBot = ys.getPixelForValue(z.min);
				const h = yBot - yTop;
				if (h <= 0) continue;

				ctx.fillStyle = z.fill;
				ctx.fillRect(left, yTop, width, h);

				if (z.stripe) {
					const sw = 8;
					ctx.save();
					ctx.beginPath();
					ctx.rect(left, yTop, width, h);
					ctx.clip();
					ctx.strokeStyle = 'rgba(239,68,68,0.18)';
					ctx.lineWidth = sw;
					ctx.beginPath();
					for (let xi = left - h; xi < right + h; xi += sw * 2.5) {
						ctx.moveTo(xi, yBot);
						ctx.lineTo(xi + h, yTop);
					}
					ctx.stroke();
					ctx.restore();
				}

				if (h > 14) {
					ctx.fillStyle = darkMode ? 'rgba(255,255,255,0.2)' : 'rgba(0,0,0,0.25)';
					ctx.font = '600 9px ui-sans-serif,system-ui,sans-serif';
					ctx.textAlign = 'right';
					ctx.textBaseline = 'middle';
					ctx.fillText(labels[i].toUpperCase(), right - 6, yTop + h / 2);
				}
			}

			ctx.restore();
		}
	};

	// Full history – only used for CSV export
	let temperatureData: Array<[number, number]> = [];
	let relaisData: Array<[number, number]> = [];
	let powerData: Array<[number, number]> = [];

	let canvas: HTMLCanvasElement;
	let downloadCSVButton: HTMLAnchorElement;
	let chart: Chart;

	// Helper – send a JSON command over the WebSocket
	const send = (cmd: string, value?: number | string) => {
		if (ws?.readyState === WebSocket.OPEN) {
			ws.send(JSON.stringify(value !== undefined ? { cmd, value } : { cmd }));
		}
	};

	onMount(() => {
		chart = new Chart(canvas, {
			type: 'line',
			plugins: [tempZonesPlugin],
			options: {
				animation: false,
				responsive: true,
				maintainAspectRatio: false,
				elements: { point: { radius: 0 } },
				plugins: {
					legend: { display: false },
					tooltip: { mode: 'nearest', intersect: false }
				},
				scales: {
					x: {
						type: 'time',
						time: {
							tooltipFormat: 'HH:mm:ss',
							displayFormats: { minute: 'HH:mm', hour: 'HH:mm' }
						},
						ticks: { color: '#7a7f96' },
						grid: { color: 'rgba(255,255,255,0.05)' }
					},
					y: {
						min: 0,
						ticks: { color: '#7a7f96' },
						grid: { color: 'rgba(255,255,255,0.05)' }
					},
					// Hidden 0-1 axis used exclusively for the relay band
					y1: {
						type: 'linear',
						display: false,
						min: 0,
						max: 1,
						position: 'right',
						grid: { drawOnChartArea: false }
					}
				}
			},
			data: {
				datasets: [
					{
						label: 'Temperature',
						data: [] as { x: number; y: number }[],
						borderColor: 'rgb(255, 99, 132)',
						fill: false,
						tension: 0.1
					} as ChartDataset<'line'>,
					{
						label: 'Target',
						data: [] as { x: number; y: number }[],
						borderColor: 'rgba(99, 132, 255, 0.6)',
						borderDash: [5, 5],
						fill: false,
						tension: 0
					} as ChartDataset<'line'>,
					{
						label: 'Relay',
						data: [] as { x: number; y: number }[],
						yAxisID: 'y1',
						borderColor: 'rgba(255, 140, 0, 0.5)',
						backgroundColor: 'rgba(255, 140, 0, 0.10)',
						fill: 'origin',
						stepped: true,
						tension: 0,
						borderWidth: 1,
						pointRadius: 0
					} as ChartDataset<'line'>
				]
			}
		});

		initWebSocket();
	});

	const initWebSocket = () => {
		if (import.meta.env.DEV) {
			console.log('DEV mode – using SpoofSocket');
			// Destroy old spoof ticker before replacing
			if (ws && 'destroy' in ws) (ws as unknown as SpoofSocket).destroy();
			ws = new SpoofSocket() as unknown as WebSocket;
		} else {
			console.log('Trying to open a WebSocket connection…');
			ws = new WebSocket(gateway);
		}
		ws.onopen = () => {
			console.log('Connection opened');
			send('getReadings');
		};
		ws.onclose = () => {
			console.log('Connection closed');
			setTimeout(initWebSocket, 2000);
		};
		ws.onmessage = onMessage;
	};

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

	const onMessage = (event: MessageEvent) => {
		const data = JSON.parse(event.data) as SensorPayload;

		mode = data.mode;
		temp = data.temperature;
		relais = data.relais;
		targetTemp = data.target_temp;
		pwmOn = data.pwm_on / 1000;
		pwmOff = data.pwm_off / 1000;
		calculatedPidOutput = data.pid;
		kp = data.kp;
		ki = data.ki;
		kd = data.kd;

		const now = Date.now();

		if (temperatureData.length >= MAX_STORED) temperatureData.shift();
		temperatureData.push([now, temp]);
		if (relaisData.length >= MAX_STORED) relaisData.shift();
		relaisData.push([now, relais]);
		if (powerData.length >= MAX_STORED) powerData.shift();
		powerData.push([now, calculatedPidOutput]);

		if (!pauseGraphUpdate && chart) {
			const tempSeries = chart.data.datasets[0].data as { x: number; y: number }[];
			const targetSeries = chart.data.datasets[1].data as { x: number; y: number }[];
			const relaySeries = chart.data.datasets[2].data as { x: number; y: number }[];

			tempSeries.push({ x: now, y: temp });
			targetSeries.push({ x: now, y: targetTemp });
			relaySeries.push({ x: now, y: data.relais });

			if (tempSeries.length > CHART_WINDOW) {
				tempSeries.shift();
				targetSeries.shift();
				relaySeries.shift();
			}

			chart.update('none');
		}
	};

	const switchRelais = () => send('switchRelais');

	const changeTargetTemp = (e: Event) => {
		const value = Number((e.target as HTMLInputElement).value);
		send('setTargetTemp', value);
	};

	const changePwmOn = (e: Event) => {
		const value = Number((e.target as HTMLInputElement).value);
		pwmOn = value;
		send('setPWMOn', value);
	};

	const changePwmOff = (e: Event) => {
		const value = Number((e.target as HTMLInputElement).value);
		pwmOff = value;
		send('setPWMOff', value);
	};

	const changeMode = (e: Event) => {
		send('setMode', (e.target as HTMLSelectElement).value);
	};

	const changeKValue = (e: Event, k: string) => {
		const value = Number((e.target as HTMLInputElement).value);
		send(`setK${k}`, value);
	};

	const prepareDownloadCSV = () => {
		let csvContent = 'Time,Temp,Relais,Power\n';
		for (let [i, row] of temperatureData.entries()) {
			csvContent += `${row[0]},${row[1]},${relaisData[i]?.[1] ?? ''},${powerData[i]?.[1] ?? ''}\n`;
		}
		const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8,' });
		const objUrl = URL.createObjectURL(blob);
		if (downloadCSVButton) {
			downloadCSVButton.classList.remove('hidden');
			downloadCSVButton.setAttribute('href', objUrl);
			downloadCSVButton.setAttribute('download', 'data.csv');
		}
	};
</script>

<div class="nav">
	<div class="nav-title">🍕 CrustCraft</div>
	<button
		class="dark-mode-btn"
		onclick={() => (darkMode = !darkMode)}
		aria-label="Toggle dark mode"
	>
		{#if darkMode}
			<Sun size={18} />
		{:else}
			<Moon size={18} />
		{/if}
	</button>
</div>

<div class="content">
	<!-- Status row -->
	<div class="card-grid">
		<div class="card">
			<p class="card-label">Temperature</p>
			<p
				class="card-value {tempIsExtreme(temp) ? 'temp-extreme' : ''}"
				style:color={tempIsExtreme(temp) ? undefined : tempColor(temp)}
			>
				{temp.toFixed(1)}<span class="card-unit">°C</span>
			</p>
			<p class="card-target">⌖ {targetTemp}°C</p>
		</div>

		<button class="card relay-card {relais ? 'relay-on' : 'relay-off'}" onclick={switchRelais}>
			<p class="card-label">Relay</p>
			<div class="onoffswitch" aria-hidden="true">
				<input
					type="checkbox"
					class="onoffswitch-checkbox"
					id="relay_toggle"
					checked={relais === 1}
					tabindex="-1"
					readonly
				/>
				<label class="onoffswitch-label" for="relay_toggle">
					<span class="onoffswitch-inner"></span>
					<span class="onoffswitch-switch"></span>
				</label>
			</div>
			<p class="card-sub">tap to toggle</p>
		</button>

		<div class="card">
			<p class="card-label">PID Output</p>
			<p class="card-value">{calculatedPidOutput.toFixed(0)}<span class="card-unit">%</span></p>
			<p class="card-sub">{(calculatedPidOutput * 12).toFixed(0)} W</p>
		</div>
	</div>

	<!-- Oven graphic -->
	<div class="oven {relais ? 'on' : ''}">
		<div class="oven-wrapper">
			<div class="oven-img-wrapper">
				<img src="/oven_small.png" alt="Oven" />
			</div>
		</div>
	</div>

	<!-- Chart -->
	<div class="chart-section">
		<canvas bind:this={canvas} id="crusty"></canvas>
	</div>

	<!-- Controls -->
	<div class="controls-grid">
		<!-- Target temp -->
		<div class="card control-card">
			<p class="card-label">Target Temperature</p>
			<div class="input-row">
				<input
					class="num-input"
					type="number"
					min="0"
					max="600"
					value={targetTemp}
					id="target_temperature"
					onchange={changeTargetTemp}
				/>
				<span class="input-unit">°C</span>
			</div>
		</div>

		<!-- Mode -->
		<div class="card control-card">
			<p class="card-label">Mode</p>
			<select class="mode-select" name="modes" id="mode_selector" onchange={changeMode}>
				<option value="pwm">PWM</option>
				<option value="pid">PID</option>
				<option value="manual" selected>Manual</option>
			</select>

			{#if mode === 'pwm'}
				<div class="subcontrol-grid mt-3">
					<div class="subcontrol">
						<label class="sublabel" for="pwm_on">ON</label>
						<div class="input-row">
							<input
								class="num-input"
								type="number"
								min="0"
								max="120"
								bind:value={pwmOn}
								id="pwm_on"
								onchange={changePwmOn}
							/>
							<span class="input-unit">s</span>
						</div>
					</div>
					<div class="subcontrol">
						<label class="sublabel" for="pwm_off">OFF</label>
						<div class="input-row">
							<input
								class="num-input"
								type="number"
								min="0"
								max="120"
								bind:value={pwmOff}
								id="pwm_off"
								onchange={changePwmOff}
							/>
							<span class="input-unit">s</span>
						</div>
					</div>
				</div>
				<p class="subinfo mt-2">
					Duty {((pwmOn / (pwmOn + pwmOff)) * 100).toFixed(0)}% &mdash; {(
						(pwmOn / (pwmOn + pwmOff)) *
						1200
					).toFixed(0)} W
				</p>
			{/if}

			{#if mode === 'pid'}
				<p class="subinfo mt-3">
					ON {pwmOn.toFixed(2)} s &nbsp;/&nbsp; OFF {pwmOff ? pwmOff.toFixed(2) : 0} s
				</p>
				<div class="subcontrol-grid mt-3">
					<div class="subcontrol">
						<label class="sublabel" for="kp">Kp</label>
						<input
							class="num-input"
							type="number"
							min="0"
							max="50"
							step="0.1"
							value={kp}
							id="kp"
							onchange={(e) => changeKValue(e, 'p')}
						/>
					</div>
					<div class="subcontrol">
						<label class="sublabel" for="ki">Ki</label>
						<input
							class="num-input"
							type="number"
							min="0"
							max="50"
							step="0.1"
							value={ki}
							id="ki"
							onchange={(e) => changeKValue(e, 'i')}
						/>
					</div>
				</div>
			{/if}
		</div>

		<!-- Data actions -->
		<div class="card control-card gap-2">
			<p class="card-label">Data</p>
			<button class="action-btn" onclick={() => (pauseGraphUpdate = !pauseGraphUpdate)}>
				{pauseGraphUpdate ? '▶ Resume' : '⏸ Pause'}
			</button>
			<button class="action-btn" onclick={prepareDownloadCSV}> ⬇ Export CSV </button>
			<a bind:this={downloadCSVButton} class="action-btn hidden" id="download_csv"> 💾 Download </a>
		</div>
	</div>
</div>
