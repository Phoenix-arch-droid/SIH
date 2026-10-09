/**
 * SIH26169: Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
 * Module: world_view.js
 * Hardware-accelerated 3D WebGL scene rendering terminal platforms, gimbal, FOV frustum,
 * beacon vectors, angular error arcs, and 3D trajectory trails.
 */

class World3DVisualizer {
  constructor(containerId) {
    this.container = document.getElementById(containerId);
    this.scene = null;
    this.camera = null;
    this.renderer = null;
    this.controls = null;

    // 3D Objects
    this.terminalPlatform = null;
    this.gimbalBase = null;
    this.gimbalHead = null;
    this.cameraFrustum = null;
    this.targetMesh = null;
    this.distractorMeshes = [];
    
    // Vectors & Visual Indicators
    this.pointingLine = null;
    this.targetLine = null;
    this.errorArc = null;
    this.groundTruthTrail = null;
    this.predictedTrail = null;

    // Trajectory history points
    this.gtPoints = [];
    this.maxTrailPoints = 120;

    this.init();
  }

  init() {
    const width = this.container.clientWidth || 800;
    const height = this.container.clientHeight || 500;

    // 1. Three.js Scene & Camera
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x06080d);
    this.scene.fog = new THREE.FogExp2(0x06080d, 0.001);

    this.camera = new THREE.PerspectiveCamera(45, width / height, 0.5, 3000);
    // Position camera in tactical isometric view
    this.camera.position.set(-60, 120, -120);

    // 2. WebGL Renderer
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(width, height);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.container.appendChild(this.renderer.domElement);

    // 3. Orbit Controls
    if (typeof THREE.OrbitControls !== 'undefined') {
      this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
      this.controls.enableDamping = true;
      this.controls.dampingFactor = 0.05;
      this.controls.target.set(0, 30, 150);
      this.controls.maxDistance = 1500;
    }

    // 4. Lighting
    const ambientLight = new THREE.AmbientLight(0x405070, 1.2);
    this.scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0xddeeff, 1.0);
    dirLight.position.set(100, 200, 100);
    this.scene.add(dirLight);

    // 5. Environment Grid & Axes
    const grid = new THREE.GridHelper(800, 40, 0x1d2738, 0x101724);
    grid.position.y = 0;
    this.scene.add(grid);

    const axes = new THREE.AxesHelper(30);
    this.scene.add(axes);

    // 6. Build 3D Models
    this._buildTerminalPlatform();
    this._buildTargetMeshes();
    this._buildVectorsAndFrustum();

    // 7. Resize Handler & Animation Loop
    window.addEventListener('resize', () => this.onResize());
    this.animate();
  }

  _buildTerminalPlatform() {
    // Mobile Terminal Host Base (Ground Station / Vehicle / Satellite chassis)
    const baseGeo = new THREE.CylinderGeometry(8, 10, 4, 16);
    const baseMat = new THREE.MeshStandardMaterial({ color: 0x1f2a3c, metalness: 0.8, roughness: 0.3 });
    this.terminalPlatform = new THREE.Mesh(baseGeo, baseMat);
    this.terminalPlatform.position.set(0, 2, 0);
    this.scene.add(this.terminalPlatform);

    // Gimbal Yoke (Pan Rotation around Y)
    const yokeGeo = new THREE.BoxGeometry(4, 6, 4);
    const yokeMat = new THREE.MeshStandardMaterial({ color: 0x334460, metalness: 0.7, roughness: 0.4 });
    this.gimbalBase = new THREE.Mesh(yokeGeo, yokeMat);
    this.gimbalBase.position.set(0, 5, 0);
    this.terminalPlatform.add(this.gimbalBase);

    // Gimbal Camera Optical Head (Tilt Rotation around X)
    const headGeo = new THREE.CylinderGeometry(2.5, 3.0, 7, 16);
    headGeo.rotateX(Math.PI / 2); // Optical axis along +Z
    const headMat = new THREE.MeshStandardMaterial({ color: 0x00e5ff, metalness: 0.5, roughness: 0.2 });
    this.gimbalHead = new THREE.Mesh(headGeo, headMat);
    this.gimbalHead.position.set(0, 4, 0);
    this.gimbalBase.add(this.gimbalHead);

    // Lens Barrel
    const lensGeo = new THREE.CylinderGeometry(2.2, 2.2, 2, 16);
    lensGeo.rotateX(Math.PI / 2);
    const lensMat = new THREE.MeshBasicMaterial({ color: 0x001122 });
    const lens = new THREE.Mesh(lensGeo, lensMat);
    lens.position.set(0, 0, 4);
    this.gimbalHead.add(lens);
  }

  _buildTargetMeshes() {
    // Designated Optical Beacon
    const beaconGeo = new THREE.SphereGeometry(3.5, 16, 16);
    const beaconMat = new THREE.MeshBasicMaterial({ color: 0x00ffff });
    this.targetMesh = new THREE.Mesh(beaconGeo, beaconMat);

    // Beacon Glow Flare
    const flareGeo = new THREE.SphereGeometry(7.0, 12, 12);
    const flareMat = new THREE.MeshBasicMaterial({
      color: 0x00e5ff,
      transparent: true,
      opacity: 0.35,
      wireframe: true
    });
    const flare = new THREE.Mesh(flareGeo, flareMat);
    this.targetMesh.add(flare);
    this.scene.add(this.targetMesh);
  }

  _buildVectorsAndFrustum() {
    // Camera Pointing Vector (Green Line along boresight)
    const pointingMat = new THREE.LineBasicMaterial({ color: 0x00e676, linewidth: 2 });
    const pointingGeo = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(0, 0, 200)
    ]);
    this.pointingLine = new THREE.Line(pointingGeo, pointingMat);
    this.scene.add(this.pointingLine);

    // Target Direction Vector (Cyan dashed line)
    const targetMat = new THREE.LineDashedMaterial({ color: 0x00e5ff, dashSize: 4, gapSize: 2 });
    const targetGeo = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(0, 50, 200)
    ]);
    this.targetLine = new THREE.Line(targetGeo, targetMat);
    this.targetLine.computeLineDistances();
    this.scene.add(this.targetLine);

    // Camera FOV Frustum Cone
    // Pyramidal wireframe representing 15-deg FOV out to range
    const frustumMat = new THREE.LineBasicMaterial({ color: 0x00e5ff, transparent: true, opacity: 0.25 });
    const r_far = 150;
    const half_w = r_far * Math.tan(THREE.MathUtils.degToRad(7.5));
    const half_h = half_w * (9 / 16);

    const fPoints = [
      new THREE.Vector3(0, 0, 0), new THREE.Vector3(-half_w,  half_h, r_far),
      new THREE.Vector3(0, 0, 0), new THREE.Vector3( half_w,  half_h, r_far),
      new THREE.Vector3(0, 0, 0), new THREE.Vector3( half_w, -half_h, r_far),
      new THREE.Vector3(0, 0, 0), new THREE.Vector3(-half_w, -half_h, r_far),
      // Base quad
      new THREE.Vector3(-half_w,  half_h, r_far), new THREE.Vector3( half_w,  half_h, r_far),
      new THREE.Vector3( half_w,  half_h, r_far), new THREE.Vector3( half_w, -half_h, r_far),
      new THREE.Vector3( half_w, -half_h, r_far), new THREE.Vector3(-half_w, -half_h, r_far),
      new THREE.Vector3(-half_w, -half_h, r_far), new THREE.Vector3(-half_w,  half_h, r_far)
    ];
    const fGeo = new THREE.BufferGeometry().setFromPoints(fPoints);
    this.cameraFrustum = new THREE.LineSegments(fGeo, frustumMat);
    this.gimbalHead.add(this.cameraFrustum);

    // Trajectory Line
    const trailMat = new THREE.LineBasicMaterial({ color: 0x00e5ff, transparent: true, opacity: 0.6 });
    const trailGeo = new THREE.BufferGeometry();
    this.groundTruthTrail = new THREE.Line(trailGeo, trailMat);
    this.scene.add(this.groundTruthTrail);
  }

  updateScene(telemetry) {
    if (!telemetry || !telemetry.objects) return;

    const obj = telemetry.objects;
    const panDeg = obj.camera_pan_deg || 0;
    const tiltDeg = obj.camera_tilt_deg || 0;

    // 1. Update Gimbal Physical Rotation
    // Pan rotates gimbalBase around Y
    // Tilt rotates gimbalHead around X
    const panRad = THREE.MathUtils.degToRad(-panDeg);
    const tiltRad = THREE.MathUtils.degToRad(tiltDeg);

    if (this.gimbalBase) this.gimbalBase.rotation.y = panRad;
    if (this.gimbalHead) this.gimbalHead.rotation.x = tiltRad;

    // 2. Update Target Beacon Position
    if (obj.targets && obj.targets.length > 0) {
      const primary = obj.targets.find(t => t.is_designated) || obj.targets[0];
      // World coordinates to Three.js: X=X, Y=Z (Up), Z=Y (Forward range)
      const tgtPos = new THREE.Vector3(primary.x, primary.z, primary.y);
      this.targetMesh.position.copy(tgtPos);

      // Trajectory Trail
      this.gtPoints.push(tgtPos.clone());
      if (this.gtPoints.length > this.maxTrailPoints) this.gtPoints.shift();
      this.groundTruthTrail.geometry.setFromPoints(this.gtPoints);

      // Target Direction Vector
      const camPos = new THREE.Vector3(0, 11, 0);
      this.targetLine.geometry.setFromPoints([camPos, tgtPos]);
      this.targetLine.computeLineDistances();

      // Update Pointing Vector Line (along boresight)
      const forward = new THREE.Vector3(0, 0, 250);
      forward.applyEuler(new THREE.Euler(tiltRad, panRad, 0, 'YXZ'));
      forward.add(camPos);
      this.pointingLine.geometry.setFromPoints([camPos, forward]);

      // Distractors
      const distractors = obj.targets.filter(t => !t.is_designated);
      // Synchronize distractor meshes
      while (this.distractorMeshes.length < distractors.length) {
        const dGeo = new THREE.SphereGeometry(2.5, 12, 12);
        const dMat = new THREE.MeshBasicMaterial({ color: 0xffa726 });
        const mesh = new THREE.Mesh(dGeo, dMat);
        this.scene.add(mesh);
        this.distractorMeshes.push(mesh);
      }
      for (let i = 0; i < this.distractorMeshes.length; i++) {
        if (i < distractors.length) {
          const d = distractors[i];
          this.distractorMeshes[i].position.set(d.x, d.z, d.y);
          this.distractorMeshes[i].visible = true;
        } else {
          this.distractorMeshes[i].visible = false;
        }
      }
    }
  }

  setCameraPreset(preset) {
    if (!this.controls) return;
    if (preset === 'tactical') {
      this.camera.position.set(-80, 140, -100);
      this.controls.target.set(0, 30, 200);
    } else if (preset === 'top') {
      this.camera.position.set(0, 450, 250);
      this.controls.target.set(0, 0, 250);
    } else if (preset === 'chase') {
      this.camera.position.set(0, 18, -35);
      this.controls.target.set(0, 15, 200);
    }
    this.controls.update();
  }

  onResize() {
    if (!this.container || !this.renderer || !this.camera) return;
    const width = this.container.clientWidth;
    const height = this.container.clientHeight;
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }

  animate() {
    requestAnimationFrame(() => this.animate());
    if (this.controls) this.controls.update();
    if (this.renderer && this.scene && this.camera) {
      this.renderer.render(this.scene, this.camera);
    }
  }
}
