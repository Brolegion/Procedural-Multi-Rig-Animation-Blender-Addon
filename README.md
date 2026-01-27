# Procedural Walk (PW) for Blender

**Procedural Walk (PW)** is a powerful animation toolkit for Blender designed to generate physically-grounded, procedural animations for armatures and simple units. It eliminates the need for manual keyframing by calculating trajectories, gait patterns, and secondary dynamics in real-time.

---

## 🚀 Key Features

* **Adaptive Walking & Running**: "Spore-like" procedural gait system with automatic IK foot placement and terrain adaptation.
* **Physics-Driven Jump & Fall**: Realistic trajectory calculations with integrated gravity, mass, and air resistance.
* **Dynamic Squash & Stretch**: Automated volume preservation for organic-feeling impacts and takeoffs.
* **Simple Unit Animations**: One-click generation for **Idle** (breathing/noise), **Stun** (dizzy/shaking), and **Death** presets (Collapse, Spin, Drama).
* **Bone & Object Versatility**: Seamlessly switch between animating the entire Object or a specific Root Bone using advanced local-to-global transform projection.
* **Secondary Dynamics**: 2nd-order spring physics for realistic lag, overshoot, and "juicy" movement.
* **FBM Noise Provider**: High-quality Fractal Brownian Motion for natural-looking procedural shakes and wobbles.

---

## 🛠 Installation

1. Download the repository as a `.zip` file.
2. In Blender, go to **Edit > Preferences > Add-ons**.
3. Click **Install...** and select the downloaded zip file.
4. Enable **Animation: Procedural Walk** in the list.

---

## 📖 How to Use

1. **Select Your Rig**: Select the Armature or Mesh you wish to animate.
2. **Configure Setup**: In the **Procedural Walk** tab in the N-panel (3D Viewport), define your leg bones (for walking/jumping) or target bone (for simple units).
3. **Adjust Parameters**: Tweak physical constants like gravity, spring stiffness, or noise intensity.
4. **Generate**: Click the desired animation button (e.g., *Generate Walk*, *Generate Jump*). The addon will automatically bake the procedural motion into keyframes.

---

## 🔬 Technical Overview

The addon utilizes several advanced mathematical concepts to ensure high-quality motion:

* **IK Anchoring**: Uses temporary trackers to keep feet "glued" to the ground during complex body maneuvers.
* **Space Projection**: Uses a custom `global_delta_to_bone_local` matrix conversion to allow world-space physics to drive bones with arbitrary rest-pose rotations.
* **Phase Sync**: Synchronizes vertical oscillation with gait cycles for believable center-of-mass movement.

---

## 📄 License

This project is licensed under the GNU GPL License - see the LICENSE file for details.

---

**Would you like me to add a section on the API for developers, or perhaps expand the "Death Presets" description?**
