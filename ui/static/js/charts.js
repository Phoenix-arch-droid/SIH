/**
 * SIH26169: Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
 * Module: charts.js
 * Synchronized live scrolling telemetry charts using Chart.js
 */

class TelemetryChartsManager {
  constructor() {
    this.maxPoints = 60;
    this.labels = [];

    // Data buffers
    this.errorDegData = [];
    this.errorMradData = [];
    this.confidenceData = [];
    this.panRateData = [];
    this.tiltRateData = [];

    this.chartError = null;
    this.chartConfidence = null;
    this.chartGimbal = null;

    this.initCharts();
  }

  initCharts() {
    if (typeof Chart === 'undefined') {
      console.warn("Chart.js not loaded");
      return;
    }

    Chart.defaults.color = '#8c9cb5';
    Chart.defaults.font.size = 10;
    Chart.defaults.borderColor = '#1d2738';

    // 1. Angular Error Chart
    const ctxErr = document.getElementById('chart-error');
    if (ctxErr) {
      this.chartError = new Chart(ctxErr, {
        type: 'line',
        data: {
          labels: this.labels,
          datasets: [
            {
              label: 'True Angular Error (mrad)',
              data: this.errorMradData,
              borderColor: '#00e5ff',
              backgroundColor: 'rgba(0, 229, 255, 0.1)',
              borderWidth: 1.5,
              tension: 0.2,
              fill: true,
              pointRadius: 0
            }
          ]
        },
        options: {
          responsive: true,
          maintainAspectRatio: false,
          animation: false,
          plugins: {
            legend: { display: false },
            tooltip: { enabled: true }
          },
          scales: {
            x: { display: false },
            y: {
              beginAtZero: true,
              grid: { color: '#162030' },
              title: { display: true, text: 'Error (mrad)', font: { size: 9 } }
            }
          }
        }
      });
    }

    // 2. Confidence & SNR Chart
    const ctxConf = document.getElementById('chart-confidence');
    if (ctxConf) {
      this.chartConfidence = new Chart(ctxConf, {
        type: 'line',
        data: {
          labels: this.labels,
          datasets: [
            {
              label: 'Confidence (%)',
              data: this.confidenceData,
              borderColor: '#00e676',
              backgroundColor: 'rgba(0, 230, 118, 0.1)',
              borderWidth: 1.5,
              tension: 0.2,
              fill: true,
              pointRadius: 0
            }
          ]
        },
        options: {
          responsive: true,
          maintainAspectRatio: false,
          animation: false,
          plugins: { legend: { display: false } },
          scales: {
            x: { display: false },
            y: {
              min: 0,
              max: 100,
              grid: { color: '#162030' },
              title: { display: true, text: 'Confidence %', font: { size: 9 } }
            }
          }
        }
      });
    }

    // 3. Gimbal Slew Rates Chart
    const ctxGimbal = document.getElementById('chart-gimbal');
    if (ctxGimbal) {
      this.chartGimbal = new Chart(ctxGimbal, {
        type: 'line',
        data: {
          labels: this.labels,
          datasets: [
            {
              label: 'Pan Slew (°/s)',
              data: this.panRateData,
              borderColor: '#b388ff',
              borderWidth: 1.2,
              pointRadius: 0
            },
            {
              label: 'Tilt Slew (°/s)',
              data: this.tiltRateData,
              borderColor: '#ffb300',
              borderWidth: 1.2,
              pointRadius: 0
            }
          ]
        },
        options: {
          responsive: true,
          maintainAspectRatio: false,
          animation: false,
          plugins: {
            legend: {
              display: true,
              labels: { boxWidth: 8, font: { size: 9 } }
            }
          },
          scales: {
            x: { display: false },
            y: {
              grid: { color: '#162030' },
              title: { display: true, text: 'deg/s', font: { size: 9 } }
            }
          }
        }
      });
    }
  }

  update(telemetry) {
    if (!telemetry) return;

    const t = telemetry.timestamp_s.toFixed(1);
    this.labels.push(t);
    this.errorDegData.push(telemetry.gt_total_error_deg);
    this.errorMradData.push(telemetry.gt_total_error_mrad);
    this.confidenceData.push(telemetry.confidence * 100);
    this.panRateData.push(telemetry.gimbal_pan_vel);
    this.tiltRateData.push(telemetry.gimbal_tilt_vel);

    if (this.labels.length > this.maxPoints) {
      this.labels.shift();
      this.errorDegData.shift();
      this.errorMradData.shift();
      this.confidenceData.shift();
      this.panRateData.shift();
      this.tiltRateData.shift();
    }

    if (this.chartError) this.chartError.update();
    if (this.chartConfidence) this.chartConfidence.update();
    if (this.chartGimbal) this.chartGimbal.update();
  }
}
