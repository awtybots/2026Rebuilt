# Awtybots 5829 V1 Goldfire for MoSim REBUILT

V1 uses the `McAllen4-shooter-fix` branch of `awtybots/2026Rebuilt`, pinned to commit `97cf46999b7ef3710ce8dbc93c62bcd31a91164f`. The branch supplies the driver controls, hub and pass RPM tables, time-of-flight table, and YAGSL and PathPlanner drivetrain settings. The two Goldfire assembly exports supply the mechanism dimensions and Fuel route. [Pinned V1 source](https://github.com/awtybots/2026Rebuilt/tree/97cf46999b7ef3710ce8dbc93c62bcd31a91164f)

The [Fuel mechanism transmission](TRANSMISSION.md) records the motor, belt, and gear paths along with their shaft RPM ratios and directions.

## Local build

Goldfire V1 is installed on this Windows machine as **Awtybots REBUILT** in MoSim. The Windows download is `.cache/mosim-authoring/goldfire-assets/Goldfire-V1-Windows-0.1.15.zip`. For another Windows installation, extract the included `rebuilt_awtybots_rebuilt` folder into `%USERPROFILE%\AppData\LocalLow\CascadeStudios\MoSim\Mods` and start MoSim. Select **Goldfire V1** for the REBUILT field. The selection card uses the provided Awtybots 5829 logo in `GoldfireV1/Artwork/Awtybots-Robot-Selection.png`.

The editable Unity project is in `.cache/mosim-authoring/MoSimulator-Public`. The model and prefabs are under `Assets/Prefabs/Games/Rebuilt/Robots/Mods/AwtybotsRebuilt`. The updated Blender rig, FBX, floor collider layout, and underside preview are in `.cache/mosim-authoring/goldfire-assets/appearance-024`. The hopper, four shooter side plates, angled intake plate, main bottom plate, both lower plates, and double spindexer wall and shaft-support plate share the smoked Lexan finish. Each of the 17 panes sorts separately and uses 38% opacity as an estimate. The NEO Vortex casings use an opaque black finish. Each spindexer cone and its 4-inch omni wheel share an axle. The mesh reduction keeps the omni roller surfaces, hopper openings, panel edges, 5829 lettering, and Vortex motor housings and mounts. The climber mount uses opaque black plastic. [Goldfire team photo](https://www.thebluealliance.com/team/5829/2026)

The spawn prefab has its own alliance-color assignment, and the robot has three in-game tutorial cards for intake, auto-shoot, and passing with the stock gamepad bindings. The Unity project also contains a local Workshop draft with a 1280 × 720 CAD preview. [MoSim bumper setup](https://github.com/MoSimulator/MoSim-Docs/blob/main/Site/modding/release-prep/Bumpers.md) · [MoSim tutorial cards](https://github.com/MoSimulator/MoSim-Docs/blob/main/Site/modding/release-prep/TutorialCards.md) · [MoSim Workshop preview requirements](https://github.com/MoSimulator/MoSim-Docs/blob/main/Site/modding/release-prep/ModUploading.md)

The build uses **135 lb provisionally** for the fully equipped robot. Replace `ProvisionalMatchWeightLbs` in `Assets/Editor/GoldfireAuthoring.cs` after measuring Goldfire with its battery and bumpers, then run `GoldfireAuthoring.AssembleAndBuild` in Unity. Use `GoldfireAuthoring.BuildWindows` to rebuild the existing prefabs without regenerating them.

The automated REBUILT scene tests (`GoldfirePlaytest.AssembleAndStartFull`) verify driving, 12.25 in of slide travel, shaft rotation, center and off-center pickup, containment at the Lexan panels, and spillover above the walls. Six Fuel drops at 5 m/s stay above the plates, and the loaded hopper test keeps Fuel above the center of the floor. Auto-align rotates the intake toward the hub from a 180° error. Stationary shots score from 2.5, 3.5, and 4.3 m. Moving shots score in both lateral directions at 0.36 m/s. Passes from 3.1 and 4.9 m land within 0.4 m of their targets. The 2.0 m hub shot clears the rim but misses the opening by about 0.1 m, so it still needs tuning with a driver.

The contact checks cover the remaining driver reports. At 3.6 m/s into a three-row Fuel pile, the chassis lifts less than 4 cm and tilts less than 5°. A shot moves the robot less than 1 mm. The hopper hex crossbar collides with Fuel and the field. The stowed robot is blocked by the stock trench overhead, and the deployed robot drives under it. The L1 hook lifts the robot 0.14 m off the carpet on the low rung. A full-throttle launch reaches 3.0 m/s in 0.64 s and stops within 1 m. That matches four Vortex motors at V1's 50 A limit with its 0.25 s ramp. `GoldfirePlaytest.AssembleAndStartClimb` runs the climb by itself.

Fuel enters beneath the intake axle, rises through the gap ahead of the front bumper, and settles in the hopper. Compound box colliders follow the actual outlines of the three horizontal Lexan plates, including their swerve cutouts. Separate colliders cover the fixed and moving panels, chassis hardware, the curved spindexer wall, and the two shooter guides. The hopper stays open at the top. These surfaces contact Fuel's outer `GamePiece1` sphere. The rigid roller cores yield to the guided contact for each Fuel ball while it passes through a tight driven opening. [MoSim game-piece layers](https://github.com/MoSimulator/MoSim-Docs/blob/main/Site/modding/physics/GamePieceLayers.md)

The feed rollers carry Fuel beneath the kickers and up the curved guide between the shooter flywheel and backspin rollers. The tower launches over the hopper toward the intake, and auto-align turns the intake end toward the shot target. The outlet uses an estimated 65° release angle. Pass RPM uses the Java table with a separate simulator scale curve. Auto-aim accounts for robot translation in its flight-time lead and turn-rate command. The local Unity project can rerun the containment, pickup, and scoring tests using `GoldfirePlaytest.StartFull`.

## CAD poses

Use `5829 V1 2026 stowed.gltf` as the prefab pose and `5829 V1 2026 extended.gltf` as the travel reference. Both are glTF 2.0 with embedded Draco-compressed meshes. Use a Draco-capable importer and simplify the hardware meshes for the game. Keep the intake carriage, its rollers, the four swerve modules, and the climber hook on separate objects.

| Moving assembly | Full travel measured between poses |
| --- | --- |
| Linear slide intake carriage | 12.25 in along the rails, 12.5° downward from horizontal. In the source CAD axes, the displacement is `(0, -0.3037745, -0.0673452)` m. |
| L1 climber hook stage | 7.64 in vertically upward. In the source CAD axes, the displacement is `(0, 0, 0.1940560)` m. |

If another export is needed, retain separate parts and export both poses. Onshape GLB with **Y axis up** and **Compress** off also works. [Onshape export options](https://cad.onshape.com/help/Content/File/exporting_files.htm)

## Unity setup

1. Open the [MoSim v26 authoring project](https://github.com/MoSimulator/MoSimulator-Public/tree/d7e2f2bb2af1eb8c46cd5ff3697f25392d6004d1) in Unity `2023.2.22f1`. Use **Tools → Modding → Create Modpack Folder** with game **Rebuilt**, pack **Awtybots REBUILT**, team **5829**, name **Awtybots V1**, and **Turret** off.
2. Copy `mosim/common/AwtybotsRebuiltRobot.cs` into the generated pack folder beside its `.asmdef`. Replace `AwtybotsV1/AwtybotsV1.cs` with the V1 controller and keep Unity’s generated `.meta` file. Copy the V1 contact scripts and `AwtybotsV1WheelVisual.cs` into the same folder as the controller. Set the metadata’s team name to `Awtybots` and the robot display name to `Goldfire V1`.
3. Build the match, menu, and spawn-selection prefabs with the V1 meshes. Use the REBUILT `Drivetrain Variant` prefab so Fuel collides with the wheels. Use the Competitive Concept robot as the reference for `DriveController`, `RebuiltAutoAimController`, and Fuel rollers. Assign the chassis root as the aim rotation reference with local +Z pointing toward the intake. Configure REV MAXSwerve with 3-inch wheels, the medium 5.08:1 drive ratio, and one NEO Vortex drive motor per module. Start with the V1 PathPlanner top speed of 5.0 m/s and use the CAD for dimensions. [V1 YAGSL physical properties](https://github.com/awtybots/2026Rebuilt/blob/97cf46999b7ef3710ce8dbc93c62bcd31a91164f/src/main/deploy/swerve/modules/physicalproperties.json) · [V1 YAGSL module config](https://github.com/awtybots/2026Rebuilt/blob/97cf46999b7ef3710ce8dbc93c62bcd31a91164f/src/main/deploy/swerve/modules/frontleft.json) · [V1 PathPlanner config](https://github.com/awtybots/2026Rebuilt/blob/97cf46999b7ef3710ce8dbc93c62bcd31a91164f/src/main/deploy/pathplanner/settings.json) · [MoSim drivetrain setup](https://github.com/MoSimulator/MoSim-Docs/blob/593e32c893f8c3055f65db491a3aba2d327d681e/Site/modding/mechanisms/DriveSetup.md) · [Competitive Concept robot](https://github.com/MoSimulator/MoSimulator-Public/tree/d7e2f2bb2af1eb8c46cd5ff3697f25392d6004d1/Assets/Prefabs/Games/Rebuilt/Robots/CC)

   Goldfire weighs **115 lb without battery and bumpers**. The deploy configs record that base mass as 115 lb in YAGSL and 52.1631 kg in PathPlanner. For `DriveController` → `Real World Weight Lbs`, add the installed battery and bumper weights to 115 lb.
4. Attach `AwtybotsV1` to the match prefab. Place the intake carriage under a rail frame with local +Z pointing outward and 12.5° downward. Keep the carriage rotation aligned to that frame. Connect its `GenericJoint` to the chassis, enable linear Z travel, and lock the other axes. The controller commands 0 to 12.25 in relative to the stowed position. Tune the slide PID and set the two agitation distances within that travel. Wire intake, hopper, kicker, and flywheel `RollerGroup` components and tune their speeds and directions. V1’s Java hub and pass tables use opposite electrical signs. The mod takes both magnitudes and applies one physical Fuel exit direction through the prefab’s roller settings. Use speed-controlled flywheels so the feed gate sees spin-up.
5. Set up bumper colliders and add a `Flywheel Penalty Tagger` for each active shooter’s main Fuel-launch roller. Set the game piece layer to `GamePiece1`. [MoSim REBUILT penalty setup](https://github.com/MoSimulator/MoSim-Docs/blob/593e32c893f8c3055f65db491a3aba2d327d681e/Site/modding/release-prep/PenaltySetup.md) · [Flywheel Penalty Tagger](https://github.com/MoSimulator/MoSim-Docs/blob/593e32c893f8c3055f65db491a3aba2d327d681e/Site/modding/misc/FlywheelPenaltyTagger.md)

## MoSim conventions

The controller follows the MoSim modding docs. It sets up PIDs and scheduler callbacks in `Init()`, registers `UpdatePid` with `AddLateRecursive` for live tuning, and commands joints each physics step with `SetLinearTarget(...).withAxis(...).useAutomaticStartingOffset()`. Inputs use `MoSimInput.WasTriggered()`. The climber joint uses the linear joint guide's 9e+30 drive damper and **Lock Joint On Disable**. The hook holds the rung by collision only, like the stock Competitive Concept climber. [Scheduled Robots](https://docs.mosimulator.com/modding/api-and-scripting/scheduledrobots) · [Linear Generic Joints](https://docs.mosimulator.com/modding/physics/generic-joints/linearjoint)

There are two deliberate additions. While the rung is seated in the hook, the climber joint's Unity `massScale` weights the 1.6 kg stage like the chassis. Without it, PhysX can't lift a 55 kg chassis through such a light body. It resets to 1 whenever the hook is up or empty. The intake, indexer, and shooter contact scripts also guide Fuel along the CAD paths, because the stock roller contacts can't model Goldfire's tight openings.

`unity-editor/` holds copies of `GoldfireAuthoring.cs` and `GoldfirePlaytest.cs`. They build the prefabs and run the tests. Copy them into the Unity project's `Assets/Editor` folder.

## Play test

Set **Robot Override** in `Assets/Scenes/Rebuilt/Rebuilt.unity` to the V1 robot asset. Test these actions using the stock MoSim gamepad profile.

| Action | Stock input | V1 behavior |
| --- | --- | --- |
| Intake / Stow | LT / D-pad down | Deploy and collect, or retract while the intake roller keeps running during the hold. |
| Auto Shoot | RT | Pick the hub or pass target on press, cap translation at 40%, and wait for flywheel speed and aim. A hub shot starts its 0.5 s feed delay at 1° and maintains a 3° window. |
| Auto Pass | RB | Request a pass regardless of field zone. |
| Manual Shoot / spin only | LB / X | Run the 1,735 RPM preset with or without automatic feeding. |
| Transfer / unjam | D-pad right / left | Run the feed rollers forward or backward without the shot gate. |
| Reverse intake / agitate | A / B | Reverse the intake rollers or cycle the carriage between the two configured distances. |
| L1 climber | D-pad up (keyboard `;`) | Each press toggles the hook between up (7.64 in) and down. Raise it, drive the hook over the low rung, then press again. Lowering the hook seats the rung in the notch and lifts the robot. Pressing again while hanging raises the hook off the rung. The hook stays locked when the match disables. |

The V1 hub sequence starts agitation 1.5 s after feed begins. Passing starts immediately and manual shooting uses 2.5 s. Check these timings, Fuel clearance, and trigger release before tuning flywheel RPM and time-of-flight compensation. The stock MoSim binds **Stow** to D-pad down. Rebind it for an LB retract if desired and move **Manual Shoot** to a free button. The McAllen branch’s driver climb bindings are commented out. The mod therefore puts the climber on D-pad up, because the stock **Climb** action shares D-pad down with **Stow**.

Trench clearance depends on the intake carriage. Stowed, the sliding hopper Lexan and hex crossbar sit about 0.60 m high, so the stock trench overhead (0.564 m) blocks the robot. Deployed, the carriage drops 67 mm along its 12.5° rails and the robot fits underneath. The lowered climber hook tops out about 8 mm below the overhead. A raised hook does not fit.

For the Windows modpack, select the metadata asset and run **Tools → Modding → Build Selected Modpack (Assets + Code)**. The bundle, DLL, and `modpack.json` are written to `BuiltMods/<mod-id>/standalonewindows64`.
