import { interpolateCounts } from './kinematics.js';

export class TrajectoryPlayer {
  constructor(spec, applyCounts) {
    this.spec = spec;
    this.applyCounts = applyCounts;
    this.frame = null;
  }

  stop() { if (this.frame) cancelAnimationFrame(this.frame); this.frame = null; }

  async play(path, options = {}) {
    this.stop();
    const millisecondsPerCount = options.millisecondsPerCount ?? 3.2;
    for (let index = 1; index < path.length; index += 1) {
      const start = path[index - 1]; const end = path[index];
      const travel = Math.max(...this.spec.joints.map((joint) => Math.abs(end[joint.id] - start[joint.id])));
      const duration = Math.max(180, travel * millisecondsPerCount);
      await this.playSegment(start, end, duration);
      options.onWaypoint?.(index, end);
    }
  }

  playSegment(start, end, duration) {
    const reduced = typeof matchMedia !== 'undefined' && matchMedia('(prefers-reduced-motion: reduce)').matches;
    return new Promise((resolve, reject) => {
      const started = performance.now();
      const frame = (now) => {
        const progress = reduced ? 1 : Math.min(1, (now - started) / duration);
        const eased = progress < 0.5 ? 4 * progress ** 3 : 1 - ((-2 * progress + 2) ** 3) / 2;
        if (!this.applyCounts(interpolateCounts(this.spec, start, end, eased))) { this.frame = null; reject(new Error('Playback stopped by an invalid servo command')); return; }
        if (progress < 1) this.frame = requestAnimationFrame(frame);
        else { this.frame = null; resolve(); }
      };
      this.frame = requestAnimationFrame(frame);
    });
  }
}
