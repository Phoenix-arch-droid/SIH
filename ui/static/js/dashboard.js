/**
 * SIH26169: Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
 * Module: dashboard.js
 * Master Dashboard Controller:
 * Handles real-time polling, telemetry updates, UI controls, scenario loading,
 * disturbance injection, one-click demo flow, and benchmark modals.
 */

document.addEventListener('DOMContentLoaded', () => {
  // Initialize sub-visualizers
  const worldViz = new World3DVisualizer('three-container');
  const cameraHUD = new CameraFeedHUD('camera-hud-canvas');
  const charts = new TelemetryChartsManager();

  // State
  let isAiMode = true;
  let activeScenarioId = 1;
  let pollInterval = null;
  const cameraFeedImg = document.getElementById('camera-feed-img');

  // DOM Elements
  const stateBadge = document.getElementById('status-state-badge');
  const stateText = document.getElementById('status-state-text');
  const kpiFps = document.getElementById('kpi-fps');
  const kpiLatency = document.getElementById('kpi-latency');
  const kpiErrorMrad = document.getElementById('kpi-error-mrad');
  const kpiErrorDeg = document.getElementById('kpi-error-deg');
  const kpiLockRate = document.getElementById('kpi-lock-rate');
  const kpiAcqTime = document.getElementById('kpi-acq-time');
  const kpiConfidence = document.getElementById('kpi-confidence');

  // Gimbal Readouts
  const valPanActual = document.getElementById('val-pan-actual');
  const valTiltActual = document.getElementById('val-tilt-actual');
  const valPanCmd = document.getElementById('val-pan-cmd');
  const valTiltCmd = document.getElementById('val-tilt-cmd');
  const valGimbalSlew = document.getElementById('val-gimbal-slew');

  // Controls
  const scenarioSelect = document.getElementById('scenario-select');
  const scenarioDesc = document.getElementById('scenario-description');
  const btnAiBaseline = document.getElementById('btn-mode-baseline');
  const btnAiEnhanced = document.getElementById('btn-mode-enhanced');
  const btnPause = document.getElementById('btn-sim-pause');
  const btnReset = document.getElementById('btn-sim-reset');
  const btnOneClickDemo = document.getElementById('btn-one-click-demo');
  const btnRunBenchmark = document.getElementById('btn-run-benchmark');
  const btnExportReport = document.getElementById('btn-export-report');
  const btnExportData = document.getElementById('btn-export-data');

  // Disturbance elements
  const disturbanceBtns = document.querySelectorAll('.btn-disturb');
  const sliderTurbulence = document.getElementById('slider-turbulence');
  const sliderVibration = document.getElementById('slider-vibration');
  const sliderNoise = document.getElementById('slider-noise');
  const chkOcclusion = document.getElementById('chk-occlusion');

  // Benchmark Modal
  const modalBenchmark = document.getElementById('modal-benchmark');
  const btnCloseModal = document.getElementById('btn-close-modal');
  const benchmarkTableBody = document.getElementById('benchmark-table-body');

  // 1. Populate Scenario Selector
  async function loadScenarios() {
    try {
      const res = await fetch('/api/scenarios');
      const scenarios = await res.json();
      scenarioSelect.innerHTML = '';
      scenarios.forEach(s => {
        const opt = document.createElement('option');
        opt.value = s.id;
        opt.textContent = `${s.id}. ${s.name}`;
        scenarioSelect.appendChild(opt);
      });
      if (scenarios.length > 0) {
        scenarioDesc.textContent = scenarios[0].description;
      }
    } catch (e) {
      console.error("Failed to load scenarios:", e);
    }
  }

  // 2. Scenario Change Handler
  scenarioSelect.addEventListener('change', async () => {
    const scenId = parseInt(scenarioSelect.value, 10);
    activeScenarioId = scenId;
    try {
      const res = await fetch('/api/scenario/load', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ scenario_id: scenId })
      });
      const data = await res.json();
      // Update description
      const opt = scenarioSelect.options[scenarioSelect.selectedIndex];
      console.log(`Loaded scenario: ${data.loaded_scenario}`);
    } catch (e) {
      console.error("Failed to set scenario:", e);
    }
  });

  // 3. AI Mode Toggles
  btnAiBaseline.addEventListener('click', async () => {
    isAiMode = false;
    btnAiBaseline.classList.add('active');
    btnAiEnhanced.classList.remove('active');
    await fetch('/api/mode/toggle', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ ai_enhanced: false })
    });
  });

  btnAiEnhanced.addEventListener('click', async () => {
    isAiMode = true;
    btnAiEnhanced.classList.add('active');
    btnAiBaseline.classList.remove('active');
    await fetch('/api/mode/toggle', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ ai_enhanced: true })
    });
  });

  // 4. Sim Controls (Pause, Reset)
  let isPaused = false;
  btnPause.addEventListener('click', async () => {
    isPaused = !isPaused;
    btnPause.innerHTML = isPaused ? '▶ RESUME' : '⏸ PAUSE';
    await fetch('/api/sim/control', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ action: isPaused ? 'pause' : 'resume' })
    });
  });

  btnReset.addEventListener('click', async () => {
    await fetch('/api/sim/control', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ action: 'reset' })
    });
  });

  // 5. Disturbance Preset Handlers
  disturbanceBtns.forEach(btn => {
    btn.addEventListener('click', async () => {
      disturbanceBtns.forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      const preset = btn.dataset.preset;
      await fetch('/api/disturbance/inject', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ preset: preset })
      });
    });
  });

  // Custom Disturbance Sliders
  function updateCustomDisturbances() {
    fetch('/api/disturbance/inject', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        preset: 'CUSTOM',
        scintillation_index: parseFloat(sliderTurbulence.value),
        vibration_amplitude_deg: parseFloat(sliderVibration.value),
        gaussian_noise_std: parseFloat(sliderNoise.value),
        occlusion_active: chkOcclusion.checked
      })
    });
  }
  if (sliderTurbulence) sliderTurbulence.addEventListener('input', updateCustomDisturbances);
  if (sliderVibration) sliderVibration.addEventListener('input', updateCustomDisturbances);
  if (sliderNoise) sliderNoise.addEventListener('input', updateCustomDisturbances);
  if (chkOcclusion) chkOcclusion.addEventListener('change', updateCustomDisturbances);

  // 6. One-Click FSOC Coarse Alignment Demo
  btnOneClickDemo.addEventListener('click', async () => {
    btnOneClickDemo.disabled = true;
    btnOneClickDemo.textContent = '🚀 RUNNING DEMO...';
    try {
      await fetch('/api/demo/start', { method: 'POST' });
      scenarioSelect.value = "13";
    } catch (e) {
      console.error(e);
    }
    setTimeout(() => {
      btnOneClickDemo.disabled = false;
      btnOneClickDemo.textContent = '⚡ RUN 1-CLICK DEMO';
    }, 12000);
  });

  // 7. Automated Benchmark Runner & Modal
  btnRunBenchmark.addEventListener('click', async () => {
    btnRunBenchmark.disabled = true;
    btnRunBenchmark.textContent = 'BENCHMARKING...';
    try {
      const res = await fetch(`/api/benchmark?scenario_id=${activeScenarioId}`);
      const data = await res.json();
      renderBenchmarkResults(data);
      modalBenchmark.style.display = 'flex';
    } catch (e) {
      console.error("Benchmark failed:", e);
    } finally {
      btnRunBenchmark.disabled = false;
      btnRunBenchmark.textContent = '⚖ RUN BENCHMARK';
    }
  });

  btnCloseModal.addEventListener('click', () => {
    modalBenchmark.style.display = 'none';
  });

  function renderBenchmarkResults(data) {
    const b = data.baseline;
    const e = data.enhanced;
    benchmarkTableBody.innerHTML = `
      <tr><td>RMS Angular Error</td><td>${b.rms_error_mrad.toFixed(2)} mrad</td><td class="winner">${e.rms_error_mrad.toFixed(2)} mrad</td><td class="winner">${data.rms_reduction_pct > 0 ? '+' : ''}${data.rms_reduction_pct.toFixed(1)}%</td></tr>
      <tr><td>Mean Angular Error</td><td>${b.mean_error_deg.toFixed(4)}°</td><td class="winner">${e.mean_error_deg.toFixed(4)}°</td><td>-</td></tr>
      <tr><td>Peak Error</td><td>${b.max_error_deg.toFixed(3)}°</td><td>${e.max_error_deg.toFixed(3)}°</td><td>-</td></tr>
      <tr><td>Lock Retention Rate</td><td>${b.lock_retention_pct.toFixed(1)}%</td><td class="winner">${e.lock_retention_pct.toFixed(1)}%</td><td class="winner">+${data.lock_improvement_pct.toFixed(1)}%</td></tr>
      <tr><td>Acquisition Time</td><td>${b.acquisition_time_s || 'N/A'} s</td><td>${e.acquisition_time_s || 'N/A'} s</td><td>-</td></tr>
      <tr><td>Mean FPS</td><td>${b.fps.toFixed(1)}</td><td>${e.fps.toFixed(1)}</td><td>Real-time</td></tr>
      <tr><td>Processing Latency</td><td>${b.latency_ms.toFixed(2)} ms</td><td>${e.latency_ms.toFixed(2)} ms</td><td>${(e.latency_ms - b.latency_ms).toFixed(2)} ms</td></tr>
    `;
  }

  // 8. Export Report & Data
  btnExportReport.addEventListener('click', () => {
    window.location.href = '/api/export/report';
  });

  btnExportData.addEventListener('click', () => {
    window.location.href = '/api/export/experiment';
  });

  // 9. Camera View Preset Buttons
  document.querySelectorAll('.btn-cam-preset').forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('.btn-cam-preset').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      worldViz.setCameraPreset(btn.dataset.view);
    });
  });

  // 10. Main Telemetry Polling Loop (30 FPS)
  let frameCounter = 0;

  async function updateTelemetry() {
    try {
      const res = await fetch('/api/telemetry');
      if (!res.ok) return;
      const t = await res.json();

      // State Badge & Styling
      stateBadge.className = `status-badge state-${t.state}`;
      stateText.textContent = t.state;

      // KPIs
      kpiFps.textContent = t.fps.toFixed(0);
      kpiLatency.textContent = `${t.latency_ms.toFixed(1)} ms`;
      kpiErrorMrad.textContent = `${t.gt_total_error_mrad.toFixed(1)} mrad`;
      kpiErrorDeg.textContent = `${t.gt_total_error_deg.toFixed(3)}°`;
      kpiLockRate.textContent = `${t.metrics.lock_retention_pct.toFixed(0)}%`;
      kpiAcqTime.textContent = t.metrics.acquisition_time_s ? `${t.metrics.acquisition_time_s.toFixed(2)}s` : '--';
      kpiConfidence.textContent = `${(t.confidence * 100).toFixed(0)}%`;

      // Gimbal Cards
      valPanActual.textContent = `${t.gimbal_pan_deg.toFixed(2)}°`;
      valTiltActual.textContent = `${t.gimbal_tilt_deg.toFixed(2)}°`;
      valPanCmd.textContent = `Cmd: ${t.cmd_pan_vel.toFixed(1)}°/s`;
      valTiltCmd.textContent = `Cmd: ${t.cmd_tilt_vel.toFixed(1)}°/s`;
      valGimbalSlew.textContent = `${Math.hypot(t.gimbal_pan_vel, t.gimbal_tilt_vel).toFixed(1)} °/s`;

      // Update 3D World Scene
      worldViz.updateScene(t);

      // Update Synthetic Camera HUD
      cameraHUD.render(t);

      // Update Live Telemetry Charts (every 2 frames ~15 Hz)
      frameCounter++;
      if (frameCounter % 2 === 0) {
        charts.update(t);
        // Refresh JPEG sensor feed image
        cameraFeedImg.src = `/api/camera_frame?t=${Date.now()}`;
      }

    } catch (e) {
      // Quiet ignore on brief poll error
    }
  }

  // Start polling
  loadScenarios();
  pollInterval = setInterval(updateTelemetry, 33); // ~30 FPS
});
