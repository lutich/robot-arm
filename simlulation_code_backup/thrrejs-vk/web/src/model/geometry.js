export function createGeometryKit(THREE, colors) {
  function material(color, roughness = 0.62, metalness = 0.14) {
    return new THREE.MeshStandardMaterial({ color, roughness, metalness });
  }

  function box(name, size, color, position = [0, 0, 0]) {
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(...size), material(color));
    mesh.name = name;
    mesh.position.set(...position);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    return mesh;
  }

  function cylinder(name, radius, depth, color, position = [0, 0, 0], axis = 'X') {
    const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, depth, 32), material(color));
    mesh.name = name;
    if (axis === 'X') mesh.rotation.z = Math.PI / 2;
    if (axis === 'Z') mesh.rotation.x = Math.PI / 2;
    mesh.position.set(...position);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    return mesh;
  }

  function addOutline(mesh, color = 0x555b60) {
    mesh.add(new THREE.LineSegments(
      new THREE.EdgesGeometry(mesh.geometry, 24),
      new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.55 }),
    ));
    return mesh;
  }

  function bracketLink(name, length, width = 0.052, depth = 0.018) {
    const group = new THREE.Group();
    group.name = name;
    const plate = 0.004;
    const railLength = Math.max(0.01, length - 0.018);
    const railZ = length / 2;
    group.add(addOutline(box(`${name}_rail_left`, [plate, depth, railLength], colors.bracket, [-(width - plate) / 2, 0, railZ])));
    group.add(addOutline(box(`${name}_rail_right`, [plate, depth, railLength], colors.bracket, [(width - plate) / 2, 0, railZ])));
    group.add(box(`${name}_lower_bridge`, [width, depth, 0.006], colors.bracket, [0, 0, 0.018]));
    group.add(cylinder(`${name}_lower_bolt`, 0.0042, width + 0.006, colors.fastener, [0, 0, 0.012]));
    group.add(cylinder(`${name}_upper_bolt`, 0.0042, width + 0.006, colors.fastener, [0, 0, length - 0.01]));
    return group;
  }

  function servoAssembly(name, width = 0.052) {
    const group = new THREE.Group();
    group.name = `${name}_servo`;
    group.add(addOutline(box(`${name}_servo_body`, [0.042, 0.022, 0.038], colors.servo, [0, 0, -0.011]), 0x464b50));
    group.add(box(`${name}_servo_tab`, [0.052, 0.028, 0.005], colors.servo, [0, 0, 0.004]));
    group.add(cylinder(`${name}_horn`, 0.012, width + 0.004, colors.horn));
    group.add(cylinder(`${name}_horn_screw`, 0.004, width + 0.007, colors.fastener));
    group.add(box(`${name}_servo_label`, [0.0015, 0.014, 0.006], 0xd6a526, [-0.0217, 0, -0.012]));
    group.add(box(`${name}_servo_label_stripe`, [0.0018, 0.010, 0.0012], colors.servo, [-0.0225, 0, -0.012]));
    for (let index = 0; index < 4; index += 1) {
      const angle = Math.PI / 4 + index * Math.PI / 2;
      group.add(cylinder(`${name}_horn_bolt_${index}`, 0.0017, width + 0.008, colors.fastener, [0, Math.sin(angle) * 0.0075, Math.cos(angle) * 0.0075]));
    }
    return group;
  }

  function cableTube(name, points, color, radius = 0.0014) {
    const curve = new THREE.CatmullRomCurve3(points.map((point) => new THREE.Vector3(...point)));
    const cable = new THREE.Mesh(new THREE.TubeGeometry(curve, 24, radius, 8, false), material(color, 0.8, 0.02));
    cable.name = name;
    cable.castShadow = true;
    return cable;
  }

  function addFaceFasteners(group, name, width, positions, radius = 0.0022) {
    positions.forEach(([y, z], index) => group.add(cylinder(`${name}_${index}`, radius, width, colors.fastener, [0, y, z])));
  }

  return { addFaceFasteners, addOutline, box, bracketLink, cableTube, cylinder, servoAssembly };
}
