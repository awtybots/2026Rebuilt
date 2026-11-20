# Awtybots 5829 V3 for MoSim REBUILT

`AwtybotsV3.cs` is the Unity robot controller for V3. The V3 geometry and prefabs will come from the competition assembly. The controller uses the installed MoSim 26.6 generation of the [public authoring project](https://github.com/MoSimulator/MoSimulator-Public/commit/d7e2f2bb2af1eb8c46cd5ff3697f25392d6004d1).

## Unity setup

1. Open the MoSim project in Unity `2023.2.22f1`. Use **Tools → Modding → Create Modpack Folder** with game **Rebuilt**, mod name **Awtybots REBUILT**, team number **5829**, team name **Awtybots V3**, and **Turret** off.
2. Copy `mosim/common/AwtybotsRebuiltRobot.cs` into `Assets/Prefabs/Games/Rebuilt/Robots/Mods/Awtybots REBUILT/`, beside the modpack assembly definition. Replace the generated `AwtybotsV3.cs` in the `AwtybotsV3` subfolder with this version and keep Unity’s generated `.meta` file. Set the robot metadata’s team name to `Awtybots` and the main robot’s display name to `V3`.
3. Attach `AwtybotsV3` to the match prefab root. Use the [Competitive Concept robot](https://github.com/MoSimulator/MoSimulator-Public/tree/d7e2f2bb2af1eb8c46cd5ff3697f25392d6004d1/Assets/Prefabs/Games/Rebuilt/Robots/CC) as wiring reference for the swerve `DriveController`, `RebuiltAutoAimController`, and physical Fuel rollers. Align its aim reference with the V3 shooter’s direction of fire. Add the V3 meshes to the match, menu, and spawn-selection prefabs.
4. Wire the intake’s driven `GenericJoint`. Make its stowed local rotation zero and set its actual axis, travel in degrees, and PID. Fill the intake, hopper, kicker, and flywheel `RollerGroup` slots. Use speed-controlled flywheel components so `AtSpeed()` reflects spin-up. Set roller directions on the prefab, then tune the intake and feeder RPMs for Fuel transport. The feeder needs both roller groups and nonzero target speeds before it will run.

## First play test

- Set **Robot Override** to the V3 robot asset in `Assets/Scenes/Rebuilt/Rebuilt.unity`. Check drive and ground clearance with the intake stowed and deployed. Use the competition assembly for geometry and weigh the match-ready robot before tuning the rigidbody. The 2BallDumper Java configuration gives starting points of 3-inch wheels, 5.08:1 drive gearing, and 5.0 m/s top speed.
- LT deploys and runs the intake. The linkage stays deployed when LT is released. The MoSim **Stow** action retracts it. The stock controller profile binds **Stow** to D-pad down and **Manual Shoot** to LB. Move those bindings if matching the real robot’s LB retract control.
- RT aims for the hub in the alliance zone and for the nearest pass target elsewhere. Feeding waits for speed and alignment. The hub and pass curves use the Java shot tables as starting values. Correct the flywheel scale and individual points from MoSim test shots. Check spin-up, trigger release, and disable behavior before tuning shots on the move.
- After adding the climber geometry and testing Fuel pickup, hub shots, and passing, select the modpack asset and run **Tools → Modding → Build Selected Modpack (Assets + Code)** with Windows as the active target. The bundle, DLL, and `modpack.json` are written to `BuiltMods/<mod-id>/standalonewindows64`.
