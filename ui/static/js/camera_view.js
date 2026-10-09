/**
 * SIH26169: Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
 * Module: camera_view.js
 * Renders the simulated optical camera sensor view and high-precision tactical HUD overlays:
 * - Optical boresight reticle
 * - Detected candidate bounding box & sub-pixel centroid
 * - Kalman filter estimated state & uncertainty ellipse
 * - Motion predictor lead-angle cross
 * - Target ID, lock status, and tracking telemetry
 */

class CameraFeedHUD {
  constructor(canvasId) {
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas.getContext('2d');
    this.width = this.canvas.width = 1280;
    this.height = this.canvas.height = 720;
  }

  render(telemetry) {
    const ctx = this.ctx;
    ctx.clearRect(0, 0, this.width, this.height);

    if (!telemetry) return;

    const cx = this.width / 2;
    const cy = this.height / 2;

    // 1. Optical Crosshair / Reticle
    ctx.strokeStyle = 'rgba(0, 229, 255, 0.4)';
    ctx.lineWidth = 1;
    ctx.setLineDash([6, 6]);

    // Horizontal & vertical lines
    ctx.beginPath();
    ctx.moveTo(cx - 80, cy);
    ctx.lineTo(cx + 80, cy);
    ctx.moveTo(cx, cy - 80);
    ctx.lineTo(cx, cy + 80);
    ctx.stroke();

    // Center circular reticle
    ctx.setLineDash([]);
    ctx.beginPath();
    ctx.arc(cx, cy, 30, 0, 2 * Math.PI);
    ctx.arc(cx, cy, 60, 0, 2 * Math.PI);
    ctx.stroke();

    // Center point
    ctx.fillStyle = '#00e5ff';
    ctx.beginPath();
    ctx.arc(cx, cy, 2, 0, 2 * Math.PI);
    ctx.fill();

    // 2. Coarse Lock Boundary Ring (0.5 deg = ~42 pixels radius at 15 deg HFOV)
    const lockRadiusPixels = 42;
    ctx.strokeStyle = telemetry.is_locked ? 'rgba(0, 230, 118, 0.8)' : 'rgba(255, 179, 0, 0.35)';
    ctx.lineWidth = telemetry.is_locked ? 2 : 1;
    ctx.beginPath();
    ctx.arc(cx, cy, lockRadiusPixels, 0, 2 * Math.PI);
    ctx.stroke();

    // 3. Ground Truth Marker (Thin Magenta Circle for Evaluation Debugging)
    if (telemetry.gt_pixel_u !== null && telemetry.gt_pixel_v !== null) {
      const gtu = telemetry.gt_pixel_u;
      const gtv = telemetry.gt_pixel_v;

      ctx.strokeStyle = 'rgba(255, 64, 129, 0.6)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.arc(gtu, gtv, 12, 0, 2 * Math.PI);
      ctx.stroke();

      ctx.fillStyle = 'rgba(255, 64, 129, 0.8)';
      ctx.font = '10px monospace';
      ctx.fillText('TRUTH', gtu + 14, gtv - 4);
    }

    // 4. Detected Candidate Marker
    if (telemetry.detected_u !== null && telemetry.detected_v !== null) {
      const du = telemetry.detected_u;
      const dv = telemetry.detected_v;

      ctx.strokeStyle = '#00e5ff';
      ctx.lineWidth = 1.5;
      const boxSize = 24;
      ctx.strokeRect(du - boxSize / 2, dv - boxSize / 2, boxSize, boxSize);

      ctx.fillStyle = '#00e5ff';
      ctx.font = 'bold 11px monospace';
      ctx.fillText(`DET [${du.toFixed(0)}, ${dv.toFixed(0)}]`, du + 16, dv + 14);
    }

    // 5. Kalman Tracked Target & Uncertainty Ellipse
    if (telemetry.tracked_u !== null && telemetry.tracked_v !== null) {
      const tu = telemetry.tracked_u;
      const tv = telemetry.tracked_v;

      // 3-Sigma Uncertainty Ellipse
      const major = Math.max(8, telemetry.uncert_major || 12);
      const minor = Math.max(6, telemetry.uncert_minor || 8);
      ctx.strokeStyle = telemetry.is_locked ? 'rgba(0, 230, 118, 0.7)' : 'rgba(0, 229, 255, 0.6)';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.ellipse(tu, tv, major, minor, 0, 0, 2 * Math.PI);
      ctx.stroke();

      // Diamond tracking marker
      ctx.fillStyle = telemetry.is_locked ? '#00e676' : '#00e5ff';
      ctx.beginPath();
      ctx.moveTo(tu, tv - 6);
      ctx.lineTo(tu + 6, tv);
      ctx.lineTo(tu, tv + 6);
      ctx.lineTo(tu - 6, tv);
      ctx.closePath();
      ctx.fill();

      // Tracking label
      ctx.font = 'bold 11px monospace';
      ctx.fillText(`BEACON_PRIMARY [${(telemetry.confidence * 100).toFixed(0)}%]`, tu + 16, tv - 8);
    }

    // 6. Motion Predictor Lead-Point (Amber Cross)
    if (telemetry.predicted_u !== null && telemetry.predicted_v !== null) {
      const pu = telemetry.predicted_u;
      const pv = telemetry.predicted_v;

      ctx.strokeStyle = '#ffb300';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(pu - 8, pv);
      ctx.lineTo(pu + 8, pv);
      ctx.moveTo(pu, pv - 8);
      ctx.lineTo(pu, pv + 8);
      ctx.stroke();

      ctx.fillStyle = '#ffb300';
      ctx.font = '10px monospace';
      ctx.fillText('PRED (+100ms)', pu + 10, pv + 12);
    }

    // 7. Tactical Corner HUD Overlays
    ctx.fillStyle = '#fff';
    ctx.font = '11px monospace';
    
    // Top-Left: Status & Errors
    ctx.fillText(`STATE: ${telemetry.state}`, 14, 22);
    ctx.fillText(`ANG ERR: ${telemetry.gt_total_error_deg.toFixed(3)}° (${telemetry.gt_total_error_mrad.toFixed(1)} mrad)`, 14, 38);
    ctx.fillText(`AZ / EL: ${telemetry.gt_az_deg.toFixed(2)}° / ${telemetry.gt_el_deg.toFixed(2)}°`, 14, 54);

    // Top-Right: Range & Scintillation
    const rangeStr = `RANGE: ${telemetry.gt_range_m.toFixed(0)} m`;
    ctx.fillText(rangeStr, this.width - ctx.measureText(rangeStr).width - 14, 22);
    const distStr = `DISTURBANCE: ${telemetry.disturbance_level}`;
    ctx.fillText(distStr, this.width - ctx.measureText(distStr).width - 14, 38);

    // Occlusion Banner Alert
    if (telemetry.is_occluded) {
      ctx.fillStyle = 'rgba(255, 61, 87, 0.85)';
      ctx.fillRect(cx - 160, 40, 320, 32);
      ctx.fillStyle = '#fff';
      ctx.font = 'bold 13px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('WARNING: LINE-OF-SIGHT OCCLUSION', cx, 61);
      ctx.textAlign = 'left';
    }
  }
}
