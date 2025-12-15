<script lang="ts">
	import { onMount } from 'svelte';

	import Chart from 'chart.js/auto';
	import 'chartjs-adapter-date-fns';

	import annotationPlugin from 'chartjs-plugin-annotation';

	import { nl } from 'date-fns/locale';

	import { pad } from '$lib/utils';
	import { page } from '$app/state';

	let gateway = `ws://${page.url.host}/ws`;
	let ws: WebSocket;

	const now = new Date();

	let mode = $state('off');
	let temp = $state(300);
	let relais = $state(0);
	let targetTemp = $state(300);
	let setOverShoot = 20;
	let setUnderShoot = 20;
	let targetTempOverShoot = $state(targetTemp);
	let targetTempUnderShoot = $state(targetTemp);
	let calculatedPidOutput = $state(0);

	let lastSwitch = new Date();
	let lastSwitchDuration = 0;

	let canvas: HTMLCanvasElement;

	let pauseGraphUpdate = $state(false);
	let spoofInterval = 250;
	let speedFactor = 250 / spoofInterval;
	const switchDelay = (30 / speedFactor) * 1000;
	let pwmOn = $state(4);
	let pwmOnDelay = (4 / speedFactor) * 1000;
	let pwmOff = $state(7);
	let pwmOffDelay = (7 / speedFactor) * 1000;

	let maxValues = 4 * 2 * 60 * 60; // 120 minutes (4 values/sec)

	let temperatureData: Array<[number, number]> = [];
	let relaisData: Array<[number, number]> = [];

	let downloadCSVButton: HTMLElement;
	let modeSpan: HTMLElement;

	const getRelaisBands = () => {
		let relaisBands = {};
		let active = false;
		let count = 0;

		for (let r of relaisData) {
			let key = 'relais' + count;
			if (r[1] === 1 && !active) {
				active = true;
				relaisBands[key] = {
					type: 'box',
					backgroundColor: 'rgba(255, 0, 0, 0.1)',
					borderWidth: 1,
					borderColor: 'rgba(255, 0, 0, 0.5)',
					drawTime: 'beforeDraw',

					xMax: relaisData[relaisData.length - 1][0] + 10,
					xMin: r[0]
				};
			}
			if (r[1] === 0 && active) {
				relaisBands[key]['xMax'] = r[0];
				active = false;
				count++;
			}
		}
		return relaisBands;
	};

	const getAnnotations = () => {
		return {
			annotations: {
				limitGreen: {
					type: 'box',
					drawTime: 'beforeDraw',
					backgroundColor: 'rgba(0, 255, 133, 0.25)',
					borderWidth: 0,
					yMax: 0,
					yMin: 350
				},
				limitOrange: {
					type: 'box',
					drawTime: 'beforeDraw',
					backgroundColor: 'rgba(255, 138, 0 ,0.25)',
					borderWidth: 0,
					yMax: 350,
					yMin: 450
				},
				limitRed: {
					type: 'box',
					drawTime: 'beforeDraw',

					backgroundColor: 'rgba(255, 0, 0, 0.25)',
					borderWidth: 0,
					yMax: 450,
					yMin: 550
				},
				targetTemp: {
					type: 'line',
					drawTime: 'beforeDraw',
					borderColor: 'black',
					borderWidth: 2,
					scaleID: 'y',
					value: targetTemp,
					borderDash: [6, 6],
					borderDashOffset: 0,
					label: {
						backgroundColor: 'transparent',
						display: true,
						color: 'rgba(0,0,0, 1)',
						content: 'Target temp',
						position: 'center',
						yAdjust: -9
					}
				},
				...getRelaisBands()
			}
		};
	};

	let chart: Chart;
	Chart.register(annotationPlugin);

	onMount(() => {
		initWebSocket();

		chart = new Chart(canvas, {
			type: 'line',
			options: {
				plugins: {
					legend: {
						display: false
					},
					tooltip: {
						mode: 'nearest',
						intersect: false,
						animation: false
					},
					annotation: getAnnotations()
				},
				scales: {
					x: {
						type: 'time',
						time: {
							tooltipFormat: 'HH:mm:ss',

							displayFormats: {
								millisecond: 'HH:mm:ss.S',
								second: 'HH:mm:ss',
								minute: 'HH:mm',
								hour: 'HH:mm'
							}
						},
						adapters: {
							date: {
								locale: nl
							}
						}
					},
					y: {
						min: 0
					}
				}
			},
			data: {
				labels: temperatureData.map((row) => row[0]),
				datasets: [
					{
						label: 'Temperature',
						data: temperatureData.map((row) => row[1]),
						fill: true,
						tension: 0.25
					}
				]
			}
		});

		if (import.meta.env.DEV) {
			// spoof some test data
			setInterval(() => {
				if (temperatureData.length > maxValues) {
					temperatureData.shift();
					relaisData.shift();
				}
				let d = new Date();

				if (temp < targetTemp / 2) {
					temp = temp + Math.random() * 0.5 - 0.125;
				} else if (temp < targetTemp / (3 / 2)) {
					temp = temp + Math.random() * 0.5 - 0.15;
				} else {
					temp = temp + Math.random() * 0.5 - 0.25;
				}

				let shootFactor = Math.min(
					((d.getTime() - lastSwitch.getTime()) / speedFactor -
						(lastSwitchDuration * 0.25) / speedFactor) /
						((3 * 60 * 1000) / speedFactor),
					1
				); // 5mins
				shootFactor = shootFactor;

				if (relais == 0) {
					temp = temp - 0.3 * shootFactor;
				} else {
					temp = temp + 0.3 * shootFactor;
				}

				let mod = temp % 0.25;
				if (mod > 0.125) {
					temp = temp - mod + 0.25;
				} else {
					temp = temp - mod;
				}

				let derivedShootFactor = Math.min(
					(d.getTime() - lastSwitch.getTime()) / speedFactor / ((3 * 60 * 1000) / speedFactor),
					1
				); // 5mins

				let overshoot = setOverShoot * derivedShootFactor; // derive over 5 mins -> 1 min eq. 3°C, 5mins eq. 15°C
				let dirivedOvershoot = Math.min(overshoot, setOverShoot);

				let undershoot = setUnderShoot * derivedShootFactor; // derive over 5 min -> 1 min eq. 3°C, 5mins eq. 15°C
				let dirivedUndershoot = Math.min(undershoot, setUnderShoot);

				targetTempOverShoot = targetTemp - dirivedOvershoot;
				targetTempUnderShoot = targetTemp + dirivedUndershoot;

				if (mode === 'auto_switch') {
					if (d.getTime() - lastSwitch.getTime() > switchDelay) {
						if (temp > targetTempOverShoot) {
							// > 270 °C
							if (relais == 1) {
								relais = 0;

								targetTempOverShoot = targetTemp;
								targetTempUnderShoot = targetTemp;
								lastSwitchDuration = d.getTime() - lastSwitch.getTime();
								lastSwitch = new Date();
							}
						}
					}
					if (d.getTime() - lastSwitch.getTime() > switchDelay) {
						if (temp < targetTempUnderShoot) {
							// < 310°C
							if (relais == 0) {
								relais = 1;

								targetTempOverShoot = targetTemp;
								targetTempUnderShoot = targetTemp;
								lastSwitchDuration = d.getTime() - lastSwitch.getTime();
								lastSwitch = new Date();
							}
						}
					}
				}

				if (mode === 'pwm') {
					if (d.getTime() - lastSwitch.getTime() > pwmOnDelay) {
						if (relais) {
							relais = 0;
							lastSwitch = new Date();
						}
					}
					if (d.getTime() - lastSwitch.getTime() > pwmOffDelay) {
						if (!relais) {
							relais = 1;
							lastSwitch = new Date();
						}
					}
				}

				temperatureData.push([d.getTime(), temp]);
				relaisData.push([d.getTime(), relais]);

				if (!pauseGraphUpdate) {
					chart.options.plugins.annotation = getAnnotations();
					chart.data.labels.push(d.getTime());
					chart.data.datasets.forEach((dataset) => {
						dataset.data.push(temp);
					});
					chart.update('none');
				}
			}, spoofInterval);
		}
	});

	const getReadings = () => {
		ws.send('getReadings');
	};

	const initWebSocket = () => {
		if (!import.meta.env.DEV) {
			console.log('Trying to open a WebSocket connection…');
			ws = new WebSocket(gateway);
			ws.onopen = onOpen;
			ws.onclose = onClose;
			ws.onmessage = onMessage;
		}
	};

	// When websocket is established, call the getReadings() function
	const onOpen = () => {
		console.log('Connection opened');
		getReadings();
	};

	const onClose = () => {
		console.log('Connection closed');
		setTimeout(initWebSocket, 2000);
	};

	// Function that receives the message from the ESP32 with the readings
	const onMessage = (event: MessageEvent) => {
		let myObj = JSON.parse(event.data);
		let keys = Object.keys(myObj);

		for (var i = 0; i < keys.length; i++) {
			let key = keys[i];
			let element = document.getElementById(key);
			if (element) {
				element.innerHTML = myObj[key].toFixed(2);
			}
			let d = new Date();
			if (key === 'temperature') {
				if (temperatureData.length > maxValues) {
					temperatureData.shift();
				}
				temperatureData.push([d.getTime(), myObj[key]]);
				temp = myObj[key];
			} else if (key === 'relais') {
				if (relaisData.length > maxValues) {
					relaisData.shift();
				}
				relaisData.push([d.getTime(), myObj[key]]);
				if (myObj[key]) {
					relais = 1;
				} else {
					relais = 0;
				}
			} else if (key === 'target_temp') {
				targetTemp = myObj[key];
			} else if (key === 'derived_overshoot') {
				targetTempOverShoot = myObj[key];
			} else if (key === 'derived_undershoot') {
				targetTempUnderShoot = myObj[key];
			} else if (key === 'pid') {
				calculatedPidOutput = myObj[key];
			} else if (key === 'pwm_on') {
				pwmOn = Number(myObj[key]) / 1000;
			} else if (key === 'pwm_off') {
				pwmOff = Number(myObj[key]) / 1000;
			} else if (key === 'mode') {
				mode = myObj[key];
				modeSpan.innerText = mode;
			}

			if (!pauseGraphUpdate) {
				chart.options.plugins.annotation = getAnnotations();
				chart.data.labels.push(d.getTime());
				chart.data.datasets.forEach((dataset) => {
					dataset.data.push(temp);
				});
				chart.update('none');
			}
		}
	};

	const switchRelais = () => {
		console.log('Send switch relais');
		if (!import.meta.env.DEV) {
			ws.send('switchRelais');
		} else {
			// spoof some test data
			let d = new Date();
			lastSwitchDuration = d.getTime() - lastSwitch.getTime();
			lastSwitch = new Date();
			if (!relais) {
				relais = 1;
			} else {
				relais = 0;
			}
		}
	};

	const changeTargetTemp = (e: Event) => {
		const target = e.target as HTMLTextAreaElement;
		let value = Number(target?.value);
		console.log('Changed target temp to ' + value);
		if (!import.meta.env.DEV) {
			ws.send('setTargetTemp: ' + pad(value, 3));
		} else {
			targetTemp = value;
			targetTempOverShoot = value;
			targetTempUnderShoot = value;
		}
	};

	const changePwmOn = (e: Event) => {
		const target = e.target as HTMLTextAreaElement;
		let value = Number(target?.value);
		console.log('Changed pwm on to ' + value);
		pwmOn = value;
		if (!import.meta.env.DEV) {
			ws.send('setPWMOn: ' + pad(value, 3));
		} else {
			pwmOnDelay = (value / speedFactor) * 1000;
		}
	};

	const changePwmOff = (e: Event) => {
		const target = e.target as HTMLTextAreaElement;
		let value = Number(target?.value);
		console.log('Changed pwm off to ' + value);
		pwmOff = value;
		if (!import.meta.env.DEV) {
			ws.send('setPWMOff: ' + pad(value, 3));
		} else {
			pwmOffDelay = (value / speedFactor) * 1000;
		}
	};

	const changeMode = (e: Event) => {
		const target = e.target as HTMLTextAreaElement;
		let value = target?.value;
		console.log('Mode set to ' + value);
		if (!import.meta.env.DEV) {
			ws.send('setMode: ' + value);
		} else {
			mode = value;
			modeSpan.innerText = value;
		}
	};

	const prepareDownloadCSV = () => {
		let csvContent = 'Time,Temp,Relais\n';

		for (let [i, temp] of temperatureData.entries()) {
			csvContent += `${temp[0]},${temp[1]},${relaisData[i] ? relaisData[i][1] : ''}\n`;
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
	<div class="nav-title">CrustCraft</div>
</div>
<div class="content">
	<div class="card-grid">
		<div class="card">
			<p class="card-title">Sensor Temperature</p>
			<p class="reading">{temp.toFixed(2)} &deg;C</p>
		</div>
		<button class="card relais" onclick={switchRelais}>
			<p>Relais</p>
			<div class="onoffswitch">
				<input
					type="checkbox"
					name="onoffswitch"
					class="onoffswitch-checkbox h-0"
					id="relais_switch_input"
					tabindex="0"
					checked={relais === 1}
				/>
				<label class="onoffswitch-label" for="relais_switch_input">
					<span class="onoffswitch-inner"></span>
					<span class="onoffswitch-switch"></span>
				</label>
			</div>
		</button>
	</div>
	<div class="oven {relais ? 'on' : ''}" id="oven">
		<div class="oven-wrapper">
			<div class="oven-img-wrapper">
				<img src="/oven_small.png" alt="Oven" />
			</div>
		</div>
	</div>
	<div class="graph">
		<div class="plot-wrapper w-[800px]">
			<div id="plot"></div>
			<canvas bind:this={canvas} id="crusty"></canvas>
		</div>
	</div>
	<div class="card-grid settings">
		<div class="card">
			<p class="card-title">Target temperature</p>
			<p class="reading">
				<input
					class="min-w-[88px] rounded"
					type="number"
					min="0"
					max="600"
					value={targetTemp}
					id="target_temperature"
					onchange={changeTargetTemp}
				/>
				<label for="target_temperature"> &deg;C </label>
			</p>
		</div>
		<div class="card">
			<p class="card-title">
				Mode: <span bind:this={modeSpan} class="mode" id="mode_span">Manual</span>
			</p>
			<p class="reading">
				<select class="rounded" name="modes" id="mode_selector" onchange={changeMode}>
					<option value="auto_switch">Auto Switch</option>
					<option value="pwm">PWM</option>
					<option value="pid">PID</option>
					<option value="off" selected>Manual</option>
				</select>
			</p>
			{#if mode === 'pwm'}
				<div class="mt-2 flex items-center gap-15">
					<div>on</div>
					<div>off</div>
				</div>

				<div class="mt-2 flex items-center gap-1">
					<input
						class="min-w-[65px] rounded"
						type="number"
						min="0"
						max="120"
						bind:value={pwmOn}
						id="pwm_on"
						onchange={changePwmOn}
					/>
					<label for="pwm_on"> s </label>
					<input
						class="min-w-[65px] rounded"
						type="number"
						min="0"
						max="120"
						bind:value={pwmOff}
						id="pwm_off"
						onchange={changePwmOff}
					/>
					<label for="pwm_off"> s </label>
				</div>
				<div class="mt-3 flex items-center gap-1">
					{((pwmOn / (pwmOn + pwmOff)) * 100).toFixed(0)}%, {(
						(pwmOn / (pwmOn + pwmOff)) *
						1200
					).toFixed(0)}W
				</div>
			{/if}
			{#if mode === 'pid'}
				<div class="mt-3 flex items-center gap-1">
					PID: {calculatedPidOutput.toFixed(0)}% {(calculatedPidOutput * 12).toFixed(0)}W
				</div>
				<div>ON:{pwmOn.toFixed(2)}s OFF:{pwmOff.toFixed(2)}s</div>
			{/if}
		</div>
		<div class="card flex cursor-pointer gap-2">
			<button
				class="cursor-pointer rounded border p-2"
				onclick={() => (pauseGraphUpdate = !pauseGraphUpdate)}
				>{pauseGraphUpdate ? 'Resume' : 'Pause'} Graph</button
			>
			<button
				class="cursor-pointer rounded border p-2"
				id="prepare_download"
				onclick={prepareDownloadCSV}>Prepare Download</button
			>
			<a
				bind:this={downloadCSVButton}
				class="hidden cursor-pointer rounded border p-2"
				id="download_csv">Download CSV</a
			>
		</div>
	</div>
</div>
