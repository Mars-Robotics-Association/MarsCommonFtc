# "New Challenge" Button Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a "New Challenge" button to the VELOCITY_MOTOR_PF flywheel controller that randomizes the hidden plant, zeros tuning params, auto-tunes profile params, and resets the sim — turning feedforward tuning into a game.

**Architecture:** Two files are modified: `FlywheelEngine` gets a `newChallenge(targetA, targetB)` method that does the randomization/zeroing/auto-tune logic, and `FlywheelTab` gets the button wired up with UI updates. The "Show Plant" button gets an "Are you sure?" confirmation dialog.

**Tech Stack:** Java, Swing, existing `SCurveVelocity.findMaxAMax` / `findMaxJDec` for profile auto-tuning.

---

### Task 1: Add `newChallenge()` method to FlywheelEngine

**Files:**
- Modify: `ControlLab/src/main/java/org/marsroboticsassociation/controllab/flywheel/FlywheelEngine.java`

- [ ] **Step 1: Add the `newChallenge` method**

Add this method to `FlywheelEngine`. It:
1. Randomizes plantKV, plantKA, plantKS within FTC-reasonable ranges
2. Rebuilds the sim with the new plant
3. Zeros kS, kV, kA, kP (leaves cutoff alone)
4. Auto-tunes accelMax and jerkDecreasing from the new plant using SCurveVelocity
5. Leaves jerkIncreasing as-is
6. Resets sim state (coast to zero)

```java
import org.marsroboticsassociation.controllib.motion.SCurveVelocity;

/**
 * Randomize plant, zero tuning params, auto-tune profile, reset sim.
 * @param targetA the "A" target velocity for profile auto-tuning
 * @param targetB the "B" target velocity for profile auto-tuning
 */
public void newChallenge(double targetA, double targetB) {
    // 1. Randomize plant params (FTC-reasonable ranges)
    //    kV: 12.5/3000 .. 12.5/1000  (different motor speeds)
    //    kA: 12.5/4000 .. 12.5/800   (different flywheel inertias)
    //    kS: 0.3 .. 1.5              (different friction)
    double maxTps = 1000 + random.nextDouble() * 2000; // 1000..3000 TPS
    this.plantKV = 12.5 / maxTps;
    double accelDenom = 800 + random.nextDouble() * 3200; // 800..4000
    this.plantKA = 12.5 / accelDenom;
    this.plantKS = 0.3 + random.nextDouble() * 1.2; // 0.3..1.5 V

    // 2. Rebuild sim with new plant
    this.sim = new FlywheelMotorSim(plantKV, plantKA);
    this.sim.setDisturbanceVoltage(-plantKS);

    // 3. Zero tuning params (leave cutoff alone)
    this.kS = 0;
    this.kV = 0;
    this.kA = 0;
    this.kP = 0;

    // 4. Auto-tune profile params from new plant
    double v0 = Math.min(targetA, targetB);
    double v1 = Math.max(targetA, targetB);
    double jInc = Double.isNaN(pfJerkIncreasing) ? 2000 : pfJerkIncreasing;
    double voltage = 12.0;

    double aMax = SCurveVelocity.findMaxAMax(v0, v1, jInc, voltage, plantKS, plantKV, plantKA);
    double jDec = SCurveVelocity.findMaxJDec(v0, v1, 0, aMax, jInc, voltage, plantKS, plantKV, plantKA);

    this.pfAccelMax = aMax;
    this.pfJerkDecreasing = jDec;

    // 5. Reset sim and rebuild controller with zeroed params
    this.elapsedSec = 0;
    this.elapsedNanos = 0;
    this.currentPower = 0;
    rebuildController();
    setTarget(0); // propagate zero target to newly built controller
}
```

Add the import for `SCurveVelocity` at the top of the file (after line 7):
```java
import org.marsroboticsassociation.controllib.motion.SCurveVelocity;
```

- [ ] **Step 2: Verify it compiles**

Run: `cd ControlLab && ../gradlew compileJava`
Expected: BUILD SUCCESSFUL

- [ ] **Step 3: Commit**

```bash
git add ControlLab/src/main/java/org/marsroboticsassociation/controllab/flywheel/FlywheelEngine.java
git commit -m "feat: add newChallenge() to FlywheelEngine for randomized plant tuning game"
```

---

### Task 2: Add "New Challenge" button and confirmation dialog to FlywheelTab

**Files:**
- Modify: `ControlLab/src/main/java/org/marsroboticsassociation/controllab/flywheel/FlywheelTab.java`

- [ ] **Step 1: Add the "New Challenge" button field**

Add a field declaration near line 40 (after `btnCoast`):
```java
private JButton btnNewChallenge;
```

- [ ] **Step 2: Create and wire the button in `buildSidebar()`**

Insert after `sidebar.add(Box.createVerticalStrut(12));` (line 88, right after the typeCombo strut), before the "Tuning" bold label:

```java
btnNewChallenge = new JButton("New Challenge");
btnNewChallenge.setMaximumSize(new Dimension(Integer.MAX_VALUE, 36));
btnNewChallenge.addActionListener(e -> onNewChallenge());
sidebar.add(btnNewChallenge);
sidebar.add(Box.createVerticalStrut(12));
```

- [ ] **Step 3: Add the `onNewChallenge()` method**

This method calls engine.newChallenge(), then updates all UI fields to reflect the new state:

```java
private void onNewChallenge() {
    engine.newChallenge(targetA, targetB);
    buffer.clear();

    // Update tuning param fields to show zeroed values
    efKS.setValue(0, "%.3f");
    efKV.setValue(0, "%.6f");
    efKA.setValue(0, "%.6f");
    efKP.setValue(0, "%.4f");

    // Update plant fields to show new (hidden) values
    efPlantKS.setValue(engine.getPlantKS(), "%.3f");
    efPlantKV.setValue(engine.getPlantKV(), "%.6f");
    efPlantKA.setValue(engine.getPlantKA(), "%.6f");

    // Update profile param fields
    efPFAccelMax.setValue(engine.getPFBAccelMax(), "%.0f");
    efPFFallingJerk.setValue(engine.getPFJerkDecreasing(), "%.0f");
}
```

- [ ] **Step 4: Make the button only visible for VELOCITY_MOTOR_PF**

In `updateControllerPanel()` (around line 243), add:
```java
btnNewChallenge.setVisible(type == FlywheelControllerType.VELOCITY_MOTOR_PF);
```

- [ ] **Step 5: Add confirmation dialog to "Show Simulator Plant" button**

Replace the existing `btnRevealPlant` action listener (lines 140-146) with one that shows an "Are you sure?" dialog before revealing:

```java
btnRevealPlant.addActionListener(e -> {
    boolean currentlyVisible = plantPanel.isVisible();
    if (!currentlyVisible) {
        // Revealing: confirm first
        int result = JOptionPane.showConfirmDialog(
                this,
                "Are you sure you want to reveal the plant values?",
                "Show Plant",
                JOptionPane.YES_NO_OPTION);
        if (result != JOptionPane.YES_OPTION) return;
    }
    boolean visible = !currentlyVisible;
    plantPanel.setVisible(visible);
    chart.getSeriesMap().get("True Velocity").setEnabled(visible);
    btnRevealPlant.setText(visible ? "Hide Simulator Plant" : "Show Simulator Plant");
    sidebar.revalidate();
});
```

- [ ] **Step 6: Verify it compiles**

Run: `cd ControlLab && ../gradlew compileJava`
Expected: BUILD SUCCESSFUL

- [ ] **Step 7: Manual smoke test**

Run: `cd ControlLab && ../gradlew run`
Verify:
1. "New Challenge" button appears only when VELOCITY_MOTOR_PF is selected
2. Clicking it zeros kS/kV/kA/kP fields and updates accelMax/fallingJerk fields
3. Chart clears and flywheel is at rest
4. "Show Simulator Plant" prompts "Are you sure?" before revealing
5. Hiding the plant does NOT prompt
6. Switching to FLYWHEEL_SIMPLE hides the "New Challenge" button

- [ ] **Step 8: Commit**

```bash
git add ControlLab/src/main/java/org/marsroboticsassociation/controllab/flywheel/FlywheelTab.java
git commit -m "feat: add New Challenge button and plant reveal confirmation dialog"
```
