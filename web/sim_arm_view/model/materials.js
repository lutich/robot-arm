// Physical materials of the arm, the props and the desk.

import * as THREE from 'three';
import { canvasTexture, noise, matTexture, servoLabelTexture } from './textures.js';

export function makeMaterials() {
  const grain = canvasTexture(256, 256, (g, w, h) => noise(g, w, h, 128, 70), { srgb: false, repeat: [3, 3] });
  const grainRough = canvasTexture(256, 256, (g, w, h) => noise(g, w, h, 150, 50), { srgb: false, repeat: [2, 2] });
  const brushed = canvasTexture(512, 64, (g, w, h) => noise(g, w, h, 205, 30, { streak: 40 }), { repeat: [1, 6] });
  const plasticGrain = canvasTexture(256, 256, (g, w, h) => noise(g, w, h, 128, 110), { srgb: false, repeat: [4, 4] });
  const boardTex = canvasTexture(512, 512, (g, w, h) => noise(g, w, h, 120, 18), { repeat: [2, 2] });
  const felt = canvasTexture(512, 512, (g, w, h) => noise(g, w, h, 205, 45, { fibres: 3000 }), { repeat: [10, 4] });
  const deskTex = canvasTexture(256, 256, (g, w, h) => noise(g, w, h, 240, 8), { repeat: [16, 8] });
  const P = (o) => new THREE.MeshPhysicalMaterial(o);
  const S = (o) => new THREE.MeshStandardMaterial(o);
  return {
    // Black anodised 2 mm aluminium brackets: satin, slightly metallic, fine grain.
    anodised: P({ color: 0x1a1c20, metalness: 0.6, roughness: 0.4, roughnessMap: grainRough, bumpMap: grain, bumpScale: 0.25, clearcoat: 0.4, clearcoatRoughness: 0.3, sheen: 1.0, sheenColor: 0x7d8da3, sheenRoughness: 0.35 }),
    // Black acrylic / anodised claw plates: a bit glossier.
    acrylic: P({ color: 0x121316, metalness: 0.35, roughness: 0.3, clearcoat: 0.6, clearcoatRoughness: 0.2, sheen: 1.0, sheenColor: 0x7d8da3, sheenRoughness: 0.35, side: THREE.DoubleSide }),
    servo: P({ color: 0x151518, roughness: 0.6, metalness: 0.0, bumpMap: plasticGrain, bumpScale: 0.35, sheen: 0.8, sheenColor: 0x5c7698, sheenRoughness: 0.5 }),
    servoCap: P({ color: 0x1b1b1f, roughness: 0.38, metalness: 0.0, clearcoat: 0.3, sheen: 0.8, sheenColor: 0x5c7698, sheenRoughness: 0.45 }),
    servoLabel: S({ map: servoLabelTexture(), roughness: 0.45 }),
    spline: S({ color: 0xd9c38a, metalness: 1, roughness: 0.3 }),
    horn: P({ color: 0xd2d5da, metalness: 1, roughness: 0.28, anisotropy: 0.5 }),
    hornBlack: P({ color: 0x1c1d21, metalness: 0.7, roughness: 0.35, clearcoat: 0.3 }),
    screw: S({ color: 0xe1e4e8, metalness: 1, roughness: 0.16 }),
    slot: S({ color: 0x24262a, metalness: 0.6, roughness: 0.6 }),
    tube: P({ color: 0xdfe2e6, metalness: 1, roughness: 0.22, map: brushed, anisotropy: 0.8 }),
    bearing: P({ color: 0xc4c8ce, metalness: 1, roughness: 0.2 }),
    pad: P({ color: 0xf3f5f6, roughness: 0.85, transparent: true, opacity: 0.85 }),
    cableBrown: P({ color: 0x5b3214, roughness: 0.45, clearcoat: 0.4 }),
    cableRed: P({ color: 0xc21d16, roughness: 0.45, clearcoat: 0.4 }),
    cableOrange: P({ color: 0xf2900f, roughness: 0.45, clearcoat: 0.4 }),
    pinkFoam: S({ color: 0xe8579f, roughness: 1 }),
    wireYellow: P({ color: 0xe7c51d, roughness: 0.45, clearcoat: 0.4 }),
    wireGreen: P({ color: 0x2f9b47, roughness: 0.45, clearcoat: 0.4 }),
    wireBlue: P({ color: 0x2d62c8, roughness: 0.45, clearcoat: 0.4 }),
    wirePurple: P({ color: 0x7b43b8, roughness: 0.45, clearcoat: 0.4 }),
    wireBlack: P({ color: 0x1a1a1a, roughness: 0.45, clearcoat: 0.4 }),
    pcbBlue: P({ color: 0x1d4f9a, roughness: 0.4, clearcoat: 0.5 }),
    pcbGreen: P({ color: 0x1f7a3a, roughness: 0.4, clearcoat: 0.5 }),
    chip: S({ color: 0x151515, roughness: 0.5 }),
    gold: S({ color: 0xe0b44c, metalness: 1, roughness: 0.25 }),
    terminal: S({ color: 0x2fa84f, roughness: 0.5 }),
    zip: P({ color: 0xf1f0ea, roughness: 0.5, transparent: true, opacity: 0.92 }),
    board: P({ color: 0x7d8186, map: boardTex, roughness: 0.55, metalness: 0.3, clearcoat: 0.15 }),
    mat: S({ map: matTexture(), roughness: 0.8 }),
    tape: P({ color: 0xffffff, roughness: 0.15, transparent: true, opacity: 0.25, clearcoat: 1 }),
    desk: S({ color: 0xf2f2ef, map: deskTex, roughness: 0.55 }),
    wall: S({ color: 0xb4b8be, map: felt, roughness: 1 }),   // light grey felt partition
    piCase: P({ color: 0xd61f4a, roughness: 0.35, clearcoat: 0.5 }),
    piWhite: S({ color: 0xf0f0f0, roughness: 0.45 }),
    metalBox: S({ color: 0x8c8f93, metalness: 0.7, roughness: 0.45 }),
    psuBody: S({ color: 0x6c7076, metalness: 0.4, roughness: 0.55 }),
    psuFront: S({ color: 0xe8e3d4, roughness: 0.6 }),
    led: S({ color: 0xff2a1a, emissive: 0xff2a1a, emissiveIntensity: 1.6 }),
    camBody: P({ color: 0x17191c, roughness: 0.4, metalness: 0.5, clearcoat: 0.4 }),
    glass: P({ color: 0x05070a, roughness: 0.05, metalness: 0.2, clearcoat: 1 }),
    sensorSilver: S({ color: 0xcfd2d6, metalness: 1, roughness: 0.3 }),
  };
}
