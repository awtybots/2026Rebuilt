using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using GameSystems.Management;
using GameSystems.Modding;
using GameSystems.Modding.Authoring.GameSystems.Modding.Editor;
using Games.Rebuilt.Components;
using Games.Rebuilt.GameManagement;
using Games.Rebuilt.Robots;
using MoSimLib;
using RobotFramework;
using RobotFramework.Components;
using RobotFramework.Controllers.Drivetrain;
using RobotFramework.Enums;
using RobotFramework.Motors;
using TMPro;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;
using Object = UnityEngine.Object;

public static class GoldfireAuthoring
{
    private const string Pack = "Assets/Prefabs/Games/Rebuilt/Robots/Mods/AwtybotsRebuilt";
    private const string Robot = Pack + "/GoldfireV1";
    private const string Model = Robot + "/Goldfire-V1-rig.fbx";
    private const string Artwork = Robot + "/Artwork";
    private const string HeroImage = Artwork + "/Goldfire-V1-Workshop-Preview.jpg";
    private const string RobotSelectionImage = Artwork + "/Awtybots-Robot-Selection.png";
    private const string TutorialTemplate = "Assets/Prefabs/UI/Components/TutorialCardTemplate.prefab";
    private const string Template = "Assets/Resources/RobotTemplates/";
    private const string DriveAsset = "Assets/Prefabs/Games/Rebuilt/Robots/Drivetrain Variant.prefab";
    private const string Field = "Assets/Scenes/Rebuilt/Rebuilt.unity";
    private const float InchesToMeters = 0.0254f;
    private const float SlideTravel = 12.25f * InchesToMeters;
    private const float ClimberTravel = 7.64f * InchesToMeters;
    private const float ClimberSliderMass = 1.6f;
    private const float ProvisionalMatchWeightLbs = 135f;
    private const float VortexFreeSpeedRpm = 6784f;
    private const float PlanetaryReduction = 3f;
    // The extended CAD belt arcs and center distances indicate a 27T motor pulley.
    private const float IntakeRollerRatio = (27f / 26f) / PlanetaryReduction;
    private const float TwindexerRatio = (28f / 28f) / PlanetaryReduction;
    private const float KickerDriveRatio = (23f / 23f) / PlanetaryReduction;
    private const float FlywheelDriveRatio = 23f / 23f;
    private const float TopBackspinRatio = FlywheelDriveRatio * (27f / 27f) * (34f / 34f);
    private const int RobotLayer = 6;
    private const int FuelLayer = 1 << 12;
    private const int FuelCollisionLayers = FuelLayer | (1 << 15);
    private const int FieldLayer = 1 << 3;

    public static void AssembleAndBuild()
    {
        Assemble();
        BuildAndValidateWindows();
    }

    public static void Assemble()
    {
        var importer = (ModelImporter)AssetImporter.GetAtPath(Model);
        if (!importer) throw new InvalidOperationException("The prepared Goldfire FBX is missing.");
        importer.bakeAxisConversion = true;
        importer.isReadable = false;
        importer.importAnimation = false;
        importer.importCameras = false;
        importer.importLights = false;
        importer.SaveAndReimport();

        var model = Load<GameObject>(Model);
        var materials = new Dictionary<Material, Material>();
        var matchPrefab = MakeMatchPrefab(model, materials);
        var menuPrefab = MakeDisplayPrefab(model, materials, "BaseRobot_Menu", "GoldfireV1 Menu");
        var spawnPrefab = MakeDisplayPrefab(model, materials, "BaseRobot_Spawn", "GoldfireV1 Spawn");
        var cards = MakeTutorialCards();
        var metadata = WriteRobotMetadata(matchPrefab, menuPrefab, spawnPrefab, cards);
        var pack = WritePack(metadata);
        WriteWorkshopDraft(pack);
        AssetDatabase.SaveAssets();
        ValidateReleaseAssets(pack);

        var field = EditorSceneManager.OpenScene(Field);
        var spawn = Object.FindFirstObjectByType<RobotSpawnController>(FindObjectsInactive.Include);
        if (!spawn)
        {
            throw new InvalidOperationException("Could not find the REBUILT Robot Spawn Controller.");
        }
        var properties = new SerializedObject(spawn);
        properties.FindProperty("robotOverride").objectReferenceValue = metadata;
        properties.ApplyModifiedPropertiesWithoutUndo();
        EditorSceneManager.SaveScene(field);
        Debug.Log($"Goldfire V1 was assembled and selected as the local field override. Pack GUID {pack.GetGuid()}");
    }

    public static void BuildAndValidateWindows()
    {
        BuildWindows();
        ValidateWindowsBundle();
    }

    public static void BuildWindows()
    {
        const string dllName = "AwtybotsRebuilt.dll";
        var pack = Load<RebuiltModpackSO>(Pack + "/AwtybotsRebuilt.asset");
        var manifest = new ModpackManifest
        {
            id = ModBuildUtility.GetModId(pack) + "_assets",
            name = pack.ModpackName,
            version = pack.ModpackVersion,
            author = pack.AuthorName,
            description = pack.Description,
            gameYear = pack.GameOption.GameYear,
            gameId = pack.GameOption.name,
            gameName = pack.GameOption.name,
            gameType = pack.GameOption.GameType.ToString(),
            modDll = dllName,
            buildHost = Application.platform.ToString(),
            modpack = new ModpackEntry { metadataGuid = pack.GetGuid(), metadataName = pack.ModpackName }
        };
        var robotGuid = pack.Robots.Single().GetGuid();
        if (!ModBuildUtility.TryBuildModpack(pack, true, out string output))
        {
            throw new InvalidOperationException("MoSim could not build the Goldfire Windows modpack.");
        }

        var platform = Path.Combine(output, "standalonewindows64");
        var bundle = Path.Combine(platform, manifest.id);
        if (!File.Exists(bundle) || new FileInfo(bundle).Length == 0)
        {
            throw new InvalidOperationException("The Windows asset bundle is missing from the build output.");
        }
        var dll = Path.Combine("Library", "ScriptAssemblies", dllName);
        if (!File.Exists(dll)) throw new InvalidOperationException("Goldfire's compiled controller DLL is missing.");
        File.Copy(dll, Path.Combine(platform, dllName), true);
        File.WriteAllText(Path.Combine(platform, "modpack.json"), JsonUtility.ToJson(manifest, true));
        Debug.Log($"Goldfire Windows package: {output}; modpack GUID {manifest.modpack.metadataGuid}; robot GUID {robotGuid}");
    }

    private static GameObject MakeMatchPrefab(GameObject model, Dictionary<Material, Material> converted)
    {
        GameObject root = UnpackTemplate("BaseRobot");
        root.name = "Goldfire V1";
        try
        {
            var visual = AddModel(model, root.transform, converted);
            var parts = visual.GetComponentsInChildren<Transform>(true).ToDictionary(
                item => item.name, item => item, StringComparer.Ordinal);
            Vector3 frontLeft = parts.Values.Single(item => item.name.Length == 8 && item.name.StartsWith("wheel_", StringComparison.Ordinal) && Corner(item.position) == "FL").position;
            Debug.Log($"Goldfire import axis check. FL={frontLeft}, BR={parts["wheel_BR"].position}");
            if (frontLeft.x > -0.25f || frontLeft.z < 0.25f || Mathf.Abs(frontLeft.y - 0.0381f) > 0.015f)
            {
                throw new InvalidOperationException("FBX wheel coordinates do not match Goldfire's measured axes and scale.");
            }
            var body = root.GetComponent<Rigidbody>();
            body.mass = (ProvisionalMatchWeightLbs * 0.45359237f) - 4.5f - ClimberSliderMass;
            body.centerOfMass = new Vector3(0f, 0.235f, -0.025f);
            body.inertiaTensorRotation = Quaternion.identity;
            body.inertiaTensor = new Vector3(2.95f, 4.65f, 2.95f);

            var drive = ConfigureDrive(root, parts);
            var robot = root.AddComponent<AwtybotsV1>();
            var input = root.GetComponent<PlayerInput>();
            input.actions = Load<InputActionAsset>("Assets/Controls/Rebuilt.inputactions");
            input.defaultActionMap = "Robot";
            input.defaultControlScheme = "Robot 1";

            var bumpers = AddChassisColliders(root);
            var settings = new SerializedObject(robot);
            var cloth = parts["bumper_cloth"].GetComponent<MeshRenderer>();
            var bumperConfig = settings.FindProperty("bumpers");
            var meshes = bumperConfig.FindPropertyRelative("bumperMeshes");
            meshes.arraySize = 1;
            meshes.GetArrayElementAtIndex(0).objectReferenceValue = cloth;
            bumperConfig.FindPropertyRelative("materialIndex").intValue = 0;
            bumperConfig.FindPropertyRelative("redBumperMaterial").objectReferenceValue = cloth.sharedMaterial;
            bumperConfig.FindPropertyRelative("blueBumperMaterial").objectReferenceValue =
                MaterialAsset("Blue Bumper Cloth", new Color(0.025f, 0.17f, 0.75f), 0.15f);
            settings.ApplyModifiedPropertiesWithoutUndo();

            var aim = root.GetComponent<RebuiltAutoAimController>();
            var aimSettings = new SerializedObject(aim);
            aimSettings.FindProperty("rotationReference").objectReferenceValue = root.transform;
            aimSettings.FindProperty("maxAlpha").floatValue = 720f;
            aimSettings.FindProperty("maxOmega").floatValue = 480f;
            aimSettings.ApplyModifiedPropertiesWithoutUndo();

            var rail = Create("Intake Rails", root.transform, Vector3.zero);
            rail.localRotation = Quaternion.Euler(12.5f, 0f, 0f);
            var carriage = Create("Intake Carriage", rail, Vector3.zero);
            parts["intake_carriage"].SetParent(carriage, true);
            var slider = MakeSlide(carriage, body);
            var climb = MakeClimber(root.transform, body, parts["climber_stage"]);

            AddFuelContainment(root.transform, carriage, parts);

            var frontRoller = MakeRoller(parts["intake_roller_1_slide"], carriage, 1f, 25.25f, 0.44f, false, false, IntakeRollerRatio);
            var intakeLinks = new[] {
                frontRoller,
                MakeRoller(parts["intake_roller_2_slide"], carriage, 1f, 25.25f, 0.31f, false, false, IntakeRollerRatio),
            };
            UseGuidedFuelContact(intakeLinks);
            var intake = MakeGroup(root.transform, "Intake Group", intakeLinks);
            AddIntakeFuelContact(root.transform, bumpers[0], frontRoller.Roller, intakeLinks[1].Roller);
            var hopperLinks = new[] { parts["indexer_left"], parts["indexer_right"] }
                .Select(part => MakeRoller(part, root.transform, 2f, 1.5f, 0.28f,
                    true, root.transform.InverseTransformPoint(part.position).x > 0f, TwindexerRatio)).ToArray();
            var hopper = MakeGroup(root.transform, "Twindexer Group", hopperLinks);
            var lowerKicker = MakeRoller(parts["kicker_lower"], root.transform, 1.5f, 15.3f, 0.25f, false, true, KickerDriveRatio * (24f / 24f));
            var kickerLinks = new[] {
                lowerKicker,
                MakeRoller(parts["kicker_powered"], root.transform, 1f, 15.3f, 0.22f, false, true, KickerDriveRatio),
                MakeRoller(parts["kicker_top"], root.transform, 1f, 15.3f, 0.22f, false, true, KickerDriveRatio * (33f / 33f)),
            };
            var kicker = MakeGroup(root.transform, "Kicker Group", kickerLinks);
            var shooterRollers = new List<RollerLink>();
            var shooterContact = new SerializedObject(Create("Shooter Outlet", root.transform, new Vector3(0f, 0.59f, -0.27f)).gameObject.AddComponent<AwtybotsV1ShooterContact>());
            shooterContact.FindProperty("fuelLayer").intValue = FuelLayer;
            foreach (var side in new[] { "left", "right" })
            {
                var primary = MakeRoller(parts["flywheel_" + side], root.transform, 2f, 7.65f, 0.30f,
                    false, true, FlywheelDriveRatio, true);
                shooterRollers.Add(primary);
                var tagger = primary.Roller.gameObject.AddComponent<FlywheelPenaltyTagger>();
                var tagSettings = new SerializedObject(tagger);
                tagSettings.FindProperty("gamePieceLayers").intValue = FuelLayer;
                tagSettings.ApplyModifiedPropertiesWithoutUndo();

                shooterRollers.Add(MakeRoller(parts["backspin_bottom_" + side], root.transform, 1f, 7f, 0.22f,
                    false, false, TopBackspinRatio * (39f / 39f)));
                var top = MakeRoller(parts["backspin_top_" + side], root.transform, 1f, 7f, 0.22f,
                    false, false, TopBackspinRatio);
                shooterRollers.Add(top);
                SetReference(shooterContact, side + "Flywheel", primary.Flywheel);
                SetReference(shooterContact, side + "Backspin", top.Roller);
            }
            shooterContact.ApplyModifiedPropertiesWithoutUndo();
            var guidedRollers = hopperLinks.Concat(kickerLinks).Concat(shooterRollers).ToArray();
            UseGuidedFuelContact(guidedRollers);
            AddIndexerFuelContact(root.transform, lowerKicker.Roller, guidedRollers);
            var shooter = MakeGroup(root.transform, "Shooter Group", shooterRollers);
            ConnectTransmissionPulleys(root, parts);

            settings = new SerializedObject(robot);
            SetReference(settings, "intakeSlide", slider);
            SetReference(settings, "climberSlide", climb);
            SetReference(settings, "intakeRollers", intake);
            SetReference(settings, "hopperRollers", hopper);
            SetReference(settings, "kickerRollers", kicker);
            SetReference(settings, "flywheelRollers", shooter);
            SetFloat(settings, "intakeRpm", VortexFreeSpeedRpm);
            SetFloat(settings, "hopperRpm", VortexFreeSpeedRpm);
            SetFloat(settings, "kickerRpm", VortexFreeSpeedRpm);
            SetFloat(settings, "agitationExtendedInches", 9.2f);
            SetFloat(settings, "agitationRetractedInches", 2.2f);
            settings.ApplyModifiedPropertiesWithoutUndo();
            if (!drive || bumpers.Length != 4)
            {
                throw new InvalidOperationException("Goldfire drivetrain or bumper geometry was not generated.");
            }

            return SavePrefab(root, "GoldfireV1");
        }
        finally
        {
            Object.DestroyImmediate(root);
        }
    }

    private static DriveController ConfigureDrive(GameObject root, IReadOnlyDictionary<string, Transform> parts)
    {
        var drive = root.GetComponent<DriveController>();
        var settings = new SerializedObject(drive);
        var dr = settings.FindProperty("drivetrainParent").objectReferenceValue as GameObject;
        if (dr)
        {
            Object.DestroyImmediate(dr);
        }
        var chassisDrive = Object.Instantiate(Load<GameObject>(DriveAsset), root.transform, false);
        chassisDrive.name = "MAXSwerve Drivetrain";
        var wheels = chassisDrive.transform.Find("Wheels");
        if (!wheels || wheels.childCount != 4)
        {
            throw new InvalidOperationException("Unexpected REBUILT drivetrain prefab hierarchy.");
        }
        wheels.localPosition = Vector3.zero;
        var wheelColliders = chassisDrive.GetComponentsInChildren<MeshCollider>(true)
            .Where(item => item.name == "Cylinder")
            .ToDictionary(item => item.transform.parent.name, item => item.transform);
        foreach (Transform wheel in wheels)
        {
            var visual = parts.Values.Single(item => item.name.Length == 8 && item.name.StartsWith("wheel_", StringComparison.Ordinal) && Corner(item.position) == wheel.name);
            wheel.position = visual.position;
            wheel.localRotation = Quaternion.identity;
            wheel.GetComponent<WheelCollider>().radius = 1.5f * InchesToMeters;
            var spin = Create(visual.name + " Roll Pivot", wheel, wheel.InverseTransformPoint(visual.position));
            visual.SetParent(spin, true);
            spin.gameObject.AddComponent<AwtybotsV1WheelVisual>();
            parts["fork_" + visual.name.Substring(6)].SetParent(wheel, true);
            if (wheelColliders.TryGetValue(wheel.name, out Transform contact))
            {
                contact.SetParent(wheel, false);
                contact.localPosition = Vector3.zero;
                contact.localScale = new Vector3(0.0762f, 0.0128f, 0.0762f);
            }
        }

        settings = new SerializedObject(drive);
        SetReference(settings, "drivetrainParent", chassisDrive);
        SetFloat(settings, "gearRatio", 5.08f);
        SetFloat(settings, "wheelDiameterInches", 3f);
        SetFloat(settings, "contactPatchWidthInches", 1.01f);
        SetFloat(settings, "realWorldWeightLbs", ProvisionalMatchWeightLbs);
        SetFloat(settings, "maxOmega", 480f);
        SetFloat(settings, "frictionCoefficient", 1.10f);
        SetFloat(settings, "biteFactor", 1.20f / 1.10f - 1f);
        SetFloat(settings, "mechanicalLossPercent", 7f);
        SetFloat(settings, "wheelRotationalInertia", 0.0012f);
        var motor = settings.FindProperty("motor");
        motor.FindPropertyRelative("motorType").enumValueIndex = (int)MotorConstants.MotorType.NeoVortex;
        var motorConfig = motor.FindPropertyRelative("config");
        motorConfig.FindPropertyRelative("K_p").floatValue = 0.5f;
        motorConfig.FindPropertyRelative("K_v").floatValue = 0.106f;
        motorConfig.FindPropertyRelative("SupplyLimit").floatValue = 50f;
        motorConfig.FindPropertyRelative("StatorLimit").floatValue = 50f;
        var speed = new SecureFloat(4.60f);
        var serializedSpeed = settings.FindProperty("maxVelocity");
        serializedSpeed.FindPropertyRelative("_obfuscationKey").intValue = speed._obfuscationKey;
        serializedSpeed.FindPropertyRelative("_encryptedValue").intValue = speed._encryptedValue;
        settings.ApplyModifiedPropertiesWithoutUndo();
        return drive;
    }

    private static BoxCollider[] AddChassisColliders(GameObject root)
    {
        var box = root.GetComponent<BoxCollider>();
        box.center = new Vector3(0f, 0.09525f, 0f);
        box.size = new Vector3(0.8806f, 0.1143f, 0.8806f);
        box.excludeLayers |= FuelCollisionLayers;
        var pieces = new[] {
            AddBox("Front Bumper", root.transform, new Vector3(0f, 0.09525f, 0.403f), new Vector3(0.8806f, 0.1143f, 0.0746f)),
            AddBox("Rear Bumper", root.transform, new Vector3(0f, 0.09525f, -0.403f), new Vector3(0.8806f, 0.1143f, 0.0746f)),
            AddBox("Left Bumper", root.transform, new Vector3(-0.403f, 0.09525f, 0f), new Vector3(0.0746f, 0.1143f, 0.7314f)),
            AddBox("Right Bumper", root.transform, new Vector3(0.403f, 0.09525f, 0f), new Vector3(0.0746f, 0.1143f, 0.7314f)),
        };
        foreach (var bumper in pieces)
        {
            bumper.tag = "Bumpers";
        }

        return pieces;
    }

    private static void AddFuelContainment(Transform chassis, Transform carriage, IReadOnlyDictionary<string, Transform> parts)
    {
        AddLexanFloors(chassis, parts);
        AddStructuralContact(chassis, carriage, parts);
        AddShooterGuideContact(chassis);
        var dividerBounds = PanelBounds(parts["chassis_smoked_pane_03"], chassis.rotation);
        AddFuelBox("Right Fixed Hopper Lexan", chassis, chassis.rotation, PanelBounds(parts["chassis_smoked_pane_01"], chassis.rotation));
        AddFuelBox("Left Fixed Hopper Lexan", chassis, chassis.rotation, PanelBounds(parts["chassis_smoked_pane_02"], chassis.rotation));
        var dividerMin = dividerBounds.min;
        // Keep the Fuel-size lower opening into both shooter lanes.
        dividerMin.y = 0.280f;
        AddFuelBox("Hopper Divider Above Feed", chassis, chassis.rotation, FromExtents(dividerMin, dividerBounds.max));
        AddFuelBox("Hopper Divider Left of Feed", chassis, chassis.rotation,
            FromExtents(dividerBounds.min, new Vector3(-0.187f, dividerMin.y, dividerBounds.max.z)));
        AddFuelBox("Hopper Divider Right of Feed", chassis, chassis.rotation,
            FromExtents(new Vector3(0.187f, dividerBounds.min.y, dividerBounds.min.z),
                new Vector3(dividerBounds.max.x, dividerMin.y, dividerBounds.max.z)));
        for (int pane = 4; pane <= 7; pane++)
        {
            var wall = parts[$"chassis_smoked_pane_{pane:00}"];
            AddShooterSidePlate(chassis, wall, pane);
        }

        AddFuelBox("Spindexer Shaft Support", chassis, chassis.rotation,
            PanelBounds(parts["chassis_spindexer_support_smoked_pane_01"], chassis.rotation));
        AddSpindexerWallContact(chassis);

        AddFuelBox("Intake Front Lexan", carriage, chassis.rotation, PanelBounds(parts["intake_carriage_smoked_pane_01"], chassis.rotation), fieldContact: true);
        AddFuelBox("Right Sliding Hopper Lexan", carriage, chassis.rotation, PanelBounds(parts["intake_carriage_smoked_pane_02"], chassis.rotation), fieldContact: true);
        AddFuelBox("Left Sliding Hopper Lexan", carriage, chassis.rotation, PanelBounds(parts["intake_carriage_smoked_pane_03"], chassis.rotation), fieldContact: true);
        AddFuelBox("Intake Overhang Lexan", carriage, chassis.rotation, PanelBounds(parts["intake_carriage_smoked_pane_04"], chassis.rotation), fieldContact: true);
        var crossbar = Create("Hopper Hex Crossbar", carriage, carriage.InverseTransformPoint(chassis.TransformPoint(new Vector3(0f, 0.584f, 0.158f))));
        crossbar.rotation = chassis.rotation;
        var shaft = crossbar.gameObject.AddComponent<CapsuleCollider>();
        shaft.direction = 0;
        shaft.radius = 0.006875f;
        shaft.height = 26.5f * InchesToMeters;
        SetContactLayers(shaft, FuelLayer | FieldLayer);

        var ramp = parts["intake_carriage_smoked_pane_05"];
        var mesh = ramp.GetComponent<MeshFilter>().sharedMesh;
        var vertices = mesh.vertices;
        var triangles = mesh.triangles;
        var surface = Vector3.zero;
        for (int index = 0; index < triangles.Length; index += 3)
        {
            Vector3 a = ramp.TransformPoint(vertices[triangles[index]]);
            Vector3 b = ramp.TransformPoint(vertices[triangles[index + 1]]);
            Vector3 c = ramp.TransformPoint(vertices[triangles[index + 2]]);
            var normal = Vector3.Cross(b - a, c - a);
            if (normal.sqrMagnitude > surface.sqrMagnitude)
                surface = normal;
        }
        var rampRotation = Quaternion.LookRotation(surface.normalized, chassis.forward);
        AddFuelBox("Angled Intake Lexan", carriage, rampRotation, PanelBounds(ramp, rampRotation));
    }

    private static void AddShooterSidePlate(Transform chassis, Transform mesh, int pane)
    {
        var bounds = PanelBounds(mesh, chassis.rotation);
        bool innerPlate = pane == 4 || pane == 6;
        // Measured leading edge by CAD slice. The open notch below the
        // kicker is where the two spindexers curl into the shooter.
        var profile = new (float top, float front)[]
        {
            (0.253f, innerPlate ? -0.160f : -0.030f),
            (0.264f, -0.002f), (0.280f, 0.021f),
            (0.326f, 0.034f), (0.349f, 0.018f),
            (0.375f, 0.002f), (0.433f, -0.018f),
            (0.481f, -0.031f), (0.508f, -0.091f),
            (bounds.max.y, -0.185f),
        };
        float bottom = bounds.min.y;
        foreach (var band in profile)
        {
            AddFuelBox($"Shooter Tower Lexan {pane - 3}", chassis, chassis.rotation,
                FromExtents(new Vector3(bounds.min.x, bottom, bounds.min.z),
                    new Vector3(bounds.max.x, band.top, band.front)));
            bottom = band.top;
        }
    }

    [Serializable]
    private sealed class LexanFloorLayout
    {
        public LexanFloorSection[] boxes;
    }

    [Serializable]
    private sealed class LexanFloorSection
    {
        public string name;
        public Vector3 min;
        public Vector3 max;
    }

    private static void AddLexanFloors(Transform chassis, IReadOnlyDictionary<string, Transform> parts)
    {
        var layout = JsonUtility.FromJson<LexanFloorLayout>(Load<TextAsset>(Robot + "/GoldfireFloorColliders.json").text);
        var floors = Create("Bottom Lexan Collision", chassis, Vector3.zero);
        foreach (var plate in layout.boxes.GroupBy(section => section.name))
        {
            var name = plate.Key switch
            {
                "chassis_bottom_smoked_pane_01" => "Main Bottom Lexan",
                "chassis_lower_smoked_pane_01" => "Lower Chassis Lexan",
                "chassis_lower_smoked_pane_02" => "Lower Battery Lexan",
                _ => throw new InvalidOperationException($"Unexpected Lexan floor: {plate.Key}"),
            };
            var assembly = Create(name, floors, Vector3.zero);
            var meshBounds = PanelBounds(parts[plate.Key], chassis.rotation);
            foreach (var section in plate)
            {
                var center = chassis.rotation * ((section.min + section.max) / 2f) + chassis.position;
                var size = section.max - section.min;
                var collider = AddBox(name, assembly, center, size);
                SetContactLayers(collider, FuelLayer);
                if (Mathf.Abs((section.max.y + Vector3.Dot(chassis.position, chassis.up)) - meshBounds.max.y) > 0.001f)
                    throw new InvalidOperationException($"Floor source for {plate.Key} is out of alignment with the model.");
            }
        }
    }

    private static void AddStructuralContact(Transform chassis, Transform carriage, IReadOnlyDictionary<string, Transform> parts)
    {
        var hardware = Create("Chassis Hardware Collision", chassis, Vector3.zero);
        foreach (float sign in new[] { -1f, 1f })
        {
            var side = sign < 0f ? "Left" : "Right";
            AddFuelBox($"{side} Frame Rail", hardware, chassis.rotation,
                new Bounds(new Vector3(sign * 0.330f, 0.0955f, 0f), new Vector3(0.025f, 0.051f, 0.508f)));
            AddFuelBox($"{side} Inner Frame Rail", hardware, chassis.rotation,
                new Bounds(new Vector3(sign * 0.038f, 0.0955f, 0f), new Vector3(0.026f, 0.051f, 0.635f)));
            AddFuelBox($"{side} Chassis Cross Rail", hardware, chassis.rotation,
                new Bounds(new Vector3(0f, 0.0955f, sign * 0.330f), new Vector3(0.508f, 0.051f, 0.025f)));
            AddFuelBox($"{side} Fixed Intake Rail", hardware, chassis.rotation,
                new Bounds(new Vector3(sign * 0.343f, 0.1825f, 0.2285f), new Vector3(0.006f, 0.225f, 0.227f)));
            AddFuelBox($"{side} Sliding Intake Rack", carriage, chassis.rotation,
                new Bounds(new Vector3(sign * 0.3335f, 0.3005f, 0.1215f), new Vector3(0.007f, 0.201f, 0.443f)));
            AddFuelBox($"{side} Limelight", hardware, chassis.rotation,
                new Bounds(new Vector3(sign * 0.276f, 0.252f, -0.276f), new Vector3(0.086f, 0.055f, 0.086f)));
            AddFuelBox($"{side} Spindexer Motor", hardware, chassis.rotation,
                new Bounds(new Vector3(sign * 0.2775f, 0.1835f, 0.0295f), new Vector3(0.076f, 0.117f, 0.079f)));
            foreach (float end in new[] { -1f, 1f })
            {
                AddFuelBox($"{side} Swerve Drive Housing", hardware, chassis.rotation,
                    new Bounds(new Vector3(sign * 0.296f, 0.1525f, end * 0.296f), new Vector3(0.067f, 0.081f, 0.062f)));
            }
        }
        AddFuelBox("Battery", hardware, chassis.rotation,
            FromExtents(new Vector3(0.236f, 0.070f, -0.207f), new Vector3(0.312f, 0.237f, -0.027f)));
        AddFuelBox("Intake Drive Motor and Gearbox", carriage, chassis.rotation,
            FromExtents(new Vector3(-0.291f, 0.249f, 0.285f), new Vector3(-0.172f, 0.322f, 0.336f)));
        foreach (var part in parts.Values.Where(item => item.name.StartsWith("chassis_black_print_mount_", StringComparison.Ordinal) ||
                     item.name.StartsWith("chassis_shooter_controller_housing_", StringComparison.Ordinal)))
        {
            AddFuelBox(part.name, hardware, chassis.rotation, PanelBounds(part, chassis.rotation));
        }
        foreach (float sign in new[] { -1f, 1f })
        {
            AddFuelBox("Shooter Flywheel Motor Bank", hardware, chassis.rotation,
                new Bounds(new Vector3(sign * 0.100f, 0.342f, -0.310f), new Vector3(0.162f, 0.075f, 0.079f)));
            AddFuelBox("Shooter Kicker Motor", hardware, chassis.rotation,
                new Bounds(new Vector3(sign * 0.118f, 0.260f, -0.296f), new Vector3(0.117f, 0.061f, 0.052f)));
        }
    }

    private static void AddShooterGuideContact(Transform chassis)
    {
        var guide = Create("Curved Shooter Guide Collision", chassis, Vector3.zero);
        var profile = new[]
        {
            new Vector2(0.127f, -0.109f), new Vector2(0.162f, -0.180f), new Vector2(0.200f, -0.215f),
            new Vector2(0.241f, -0.235f), new Vector2(0.303f, -0.246f), new Vector2(0.413f, -0.262f),
        };
        foreach (float x in new[] { -0.100f, 0.100f })
        {
            for (int segment = 1; segment < profile.Length; segment++)
            {
                Vector3 from = new(x, profile[segment - 1].x, profile[segment - 1].y);
                Vector3 to = new(x, profile[segment].x, profile[segment].y);
                Vector3 tangent = (to - from).normalized;
                Vector3 towardFuel = Vector3.Cross(Vector3.right, tangent);
                var rotation = chassis.rotation * Quaternion.LookRotation(towardFuel, tangent);
                Vector3 center = chassis.TransformPoint((from + to) / 2f - towardFuel * 0.014f);
                AddFuelBox("Shooter Guide Segment", guide, rotation,
                    new Bounds(Quaternion.Inverse(rotation) * center, new Vector3(0.162f, Vector3.Distance(to, from) + 0.004f, 0.018f)));
            }
        }
    }

    private static void AddSpindexerWallContact(Transform chassis)
    {
        var wall = Create("Spindexer Lexan Wall Collision", chassis, Vector3.zero);
        var halfProfile = new[]
        {
            new Vector2(0f, 0.334f), new Vector2(0.081f, 0.315f), new Vector2(0.159f, 0.279f),
            new Vector2(0.221f, 0.221f), new Vector2(0.254f, 0.153f), new Vector2(0.271f, 0.078f),
            new Vector2(0.272f, -0.012f),
        };
        foreach (float side in new[] { -1f, 1f })
        {
            for (int segment = 1; segment < halfProfile.Length; segment++)
            {
                var a = halfProfile[segment - 1];
                var b = halfProfile[segment];
                Vector3 from = chassis.TransformPoint(new Vector3(a.x * side, 0.1685f, a.y));
                Vector3 to = chassis.TransformPoint(new Vector3(b.x * side, 0.1685f, b.y));
                var rotation = Quaternion.LookRotation(to - from, chassis.up);
                AddFuelBox("Curved Spindexer Lexan", wall, rotation,
                    new Bounds(Quaternion.Inverse(rotation) * ((from + to) / 2f), new Vector3(0.006f, 0.083f, Vector3.Distance(from, to) + 0.004f)));
            }
        }
    }

    private static Bounds PanelBounds(Transform panel, Quaternion rotation)
    {
        var toBox = Quaternion.Inverse(rotation);
        var vertices = panel.GetComponent<MeshFilter>().sharedMesh.vertices;
        var bounds = new Bounds(toBox * panel.TransformPoint(vertices[0]), Vector3.zero);
        foreach (var vertex in vertices)
            bounds.Encapsulate(toBox * panel.TransformPoint(vertex));
        return bounds;
    }

    private static Bounds FromExtents(Vector3 min, Vector3 max) => new Bounds((min + max) / 2f, max - min);

    private static void AddFuelBox(string name, Transform parent, Quaternion rotation, Bounds bounds, bool fieldContact = false)
    {
        var collider = AddBox(name, parent, rotation * bounds.center, Vector3.Max(bounds.size, Vector3.one * 0.006f));
        collider.transform.rotation = rotation;
        var inherited = collider.transform.lossyScale;
        collider.transform.localScale = new Vector3(1f / Mathf.Abs(inherited.x), 1f / Mathf.Abs(inherited.y), 1f / Mathf.Abs(inherited.z));
        SetContactLayers(collider, fieldContact ? FuelLayer | FieldLayer : FuelLayer);
    }

    private static void SetContactLayers(Collider collider, int layers)
    {
        // Fuel's outer sphere collides with these panels. Its smaller
        // companion sphere is reserved for contact between Fuel pieces.
        collider.includeLayers = layers;
        collider.excludeLayers = ~layers;
        collider.layerOverridePriority = 1;
    }

    private static void UseGuidedFuelContact(IEnumerable<RollerLink> rollers)
    {
        foreach (var link in rollers)
        {
            var settings = new SerializedObject(link.Roller);
            // These rollers use the robot's guided Fuel contact. Exclude
            // every stock force contact so they can't grip the chassis.
            settings.FindProperty("ignoreLayers").intValue = ~0;
            settings.ApplyModifiedPropertiesWithoutUndo();
        }
    }

    private static void AddIntakeFuelContact(
        Transform chassis, BoxCollider frontBumper, InteractionRoller frontRoller, InteractionRoller upperRoller)
    {
        var contact = Create("Intake Fuel Contact", chassis, new Vector3(0f, 0.16f, 0.49f));
        var properties = new SerializedObject(contact.gameObject.AddComponent<AwtybotsV1IntakeContact>());
        properties.FindProperty("intakeRoller").objectReferenceValue = frontRoller;
        properties.FindProperty("upperRoller").objectReferenceValue = upperRoller;
        properties.FindProperty("frontBumper").objectReferenceValue = frontBumper;
        properties.FindProperty("fuelLayer").intValue = FuelLayer;
        properties.ApplyModifiedPropertiesWithoutUndo();
    }

    private static void AddIndexerFuelContact(Transform chassis, InteractionRoller kicker, IReadOnlyList<RollerLink> links)
    {
        var contact = Create("Indexer Fuel Contact", chassis, new Vector3(0f, 0.39f, 0.045f));
        var properties = new SerializedObject(contact.gameObject.AddComponent<AwtybotsV1IndexerContact>());
        properties.FindProperty("kicker").objectReferenceValue = kicker;
        var feedRollers = properties.FindProperty("feedRollers");
        feedRollers.arraySize = links.Count;
        for (int i = 0; i < links.Count; i++)
            feedRollers.GetArrayElementAtIndex(i).objectReferenceValue = links[i].Roller;
        properties.FindProperty("fuelLayer").intValue = FuelLayer;
        properties.ApplyModifiedPropertiesWithoutUndo();
    }

    private static GenericJoint MakeSlide(Transform carriage, Rigidbody chassis)
    {
        var body = carriage.gameObject.AddComponent<Rigidbody>();
        body.mass = 4.5f;
        body.useGravity = true;
        body.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;
        var constraint = carriage.gameObject.AddComponent<ConfigurableJoint>();
        constraint.connectedBody = chassis;
        constraint.autoConfigureConnectedAnchor = false;
        constraint.connectedAnchor = chassis.transform.InverseTransformPoint(
            carriage.parent.TransformPoint(new Vector3(0f, 0f, SlideTravel / 2f)));
        constraint.xMotion = ConfigurableJointMotion.Locked;
        constraint.yMotion = ConfigurableJointMotion.Locked;
        constraint.zMotion = ConfigurableJointMotion.Limited;
        constraint.angularXMotion = ConfigurableJointMotion.Locked;
        constraint.angularYMotion = ConfigurableJointMotion.Locked;
        constraint.angularZMotion = ConfigurableJointMotion.Locked;
        constraint.linearLimit = new SoftJointLimit { limit = SlideTravel / 2f, contactDistance = 0.003f };
        constraint.zDrive = new JointDrive { positionDamper = 1500f, maximumForce = 1400f };
        constraint.projectionMode = JointProjectionMode.PositionAndRotation;
        return carriage.gameObject.AddComponent<GenericJoint>();
    }

    private static GenericJoint MakeClimber(Transform chassis, Rigidbody body, Transform model)
    {
        var stage = Create("Climber Slider", chassis, Vector3.zero);
        model.SetParent(stage, true);
        var sliderBody = stage.gameObject.AddComponent<Rigidbody>();
        sliderBody.mass = ClimberSliderMass;
        sliderBody.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;

        var joint = stage.gameObject.AddComponent<ConfigurableJoint>();
        joint.connectedBody = body;
        joint.autoConfigureConnectedAnchor = false;
        joint.connectedAnchor = Vector3.up * (ClimberTravel / 2f);
        joint.xMotion = joint.zMotion = ConfigurableJointMotion.Locked;
        joint.yMotion = ConfigurableJointMotion.Limited;
        joint.angularXMotion = joint.angularYMotion = joint.angularZMotion = ConfigurableJointMotion.Locked;
        joint.linearLimit = new SoftJointLimit { limit = ClimberTravel / 2f, contactDistance = 0.002f };
        // MoSim linear joint guide: raise the drive damper to 9e+30 when the joint needs force.
        // The PID Max then limits hook speed.
        joint.yDrive = new JointDrive { positionDamper = 9e30f, maximumForce = float.MaxValue };

        AddFuelBox("Climber Stage Rail", stage, chassis.rotation,
            FromExtents(new Vector3(-0.337f, 0.174f, -0.146f), new Vector3(-0.311f, 0.504f, -0.121f)), fieldContact: true);
        foreach (float x in new[] { -0.3385f, -0.3095f })
        {
            AddFuelBox("Hook Crown", stage, chassis.rotation,
                new Bounds(new Vector3(x, 0.549f, -0.087f), new Vector3(0.006f, 0.012f, 0.124f)), fieldContact: true);
            AddFuelBox("Hook Heel", stage, chassis.rotation,
                new Bounds(new Vector3(x, 0.504f, -0.137f), new Vector3(0.006f, 0.077f, 0.025f)), fieldContact: true);
            AddFuelBox("Hook Lip", stage, chassis.rotation,
                new Bounds(new Vector3(x, 0.533f, -0.054f), new Vector3(0.006f, 0.022f, 0.026f)), fieldContact: true);
        }
        var climber = stage.gameObject.AddComponent<GenericJoint>();
        // MoSim linear joint guide: climbers lock on disable so the robot stays hung after the match.
        var climberSettings = new SerializedObject(climber);
        climberSettings.FindProperty("lockJointOnDisable").boolValue = true;
        climberSettings.ApplyModifiedPropertiesWithoutUndo();
        return climber;
    }

    private readonly struct RollerLink
    {
        public readonly InteractionRoller Roller;
        public readonly FlywheelBehaviour Flywheel;
        public readonly float Ratio;
        public readonly bool Flipped;

        public RollerLink(InteractionRoller roller, FlywheelBehaviour flywheel, float ratio, bool flipped)
        {
            Roller = roller;
            Flywheel = flywheel;
            Ratio = ratio;
            Flipped = flipped;
        }
    }

    private static RollerLink MakeRoller(
        Transform model, Transform parent, float radius, float length, float squish,
        bool vertical, bool flipped, float ratio, bool speedControlled = false)
    {
        var rollerNode = Create(model.name + " Physics", parent, parent.InverseTransformPoint(model.position));
        if (vertical)
        {
            rollerNode.rotation = Quaternion.Euler(0f, 0f, 90f);
        }
        else
        {
            rollerNode.rotation = Quaternion.identity;
        }
        // Give the CAD mesh a pivot whose local X matches the physics axle.
        var visualPivot = Create(model.name + " Visual Pivot", rollerNode, Vector3.zero);
        model.SetParent(visualPivot, true);

        var roller = rollerNode.gameObject.AddComponent<InteractionRoller>();
        var settings = new SerializedObject(roller);
        SetFloat(settings, "radius", radius);
        SetFloat(settings, "length", length);
        SetFloat(settings, "maxSquish", squish);
        SetFloat(settings, "rpm", 0f);
        if (speedControlled || model.name.StartsWith("backspin_", StringComparison.Ordinal))
        {
            SetFloat(settings, "motorSlipResponse", 0.18f);
            SetFloat(settings, "compressionStiffness", 920f);
            SetFloat(settings, "compressionDamping", 32f);
        }
        settings.FindProperty("motor").enumValueIndex = (int)(speedControlled
            ? InteractionRoller.RollerMotor.Shooter : InteractionRoller.RollerMotor.FullSize);
        AddRollerModel(settings, visualPivot);
        settings.ApplyModifiedPropertiesWithoutUndo();

        FlywheelBehaviour flywheel = null;
        if (speedControlled)
        {
            flywheel = rollerNode.gameObject.AddComponent<FlywheelBehaviour>();
            var flywheelSettings = new SerializedObject(flywheel);
            SetFloat(flywheelSettings, "m_SpinUpTime", 0.25f);
            SetFloat(flywheelSettings, "m_SpinDownTime", 0.45f);
            flywheelSettings.FindProperty("m_GamePieceLayers").intValue = FuelLayer;
            flywheelSettings.ApplyModifiedPropertiesWithoutUndo();
        }
        return new RollerLink(roller, flywheel, ratio, flipped);
    }

    private static void AddRollerModel(SerializedObject settings, Transform pivot, float speedRatio = 1f)
    {
        var models = settings.FindProperty("models");
        int index = models.arraySize++;
        var appearance = models.GetArrayElementAtIndex(index);
        appearance.FindPropertyRelative("model").objectReferenceValue = pivot.gameObject;
        appearance.FindPropertyRelative("jointAxis").enumValueIndex = (int)JointAxis.X;
        appearance.FindPropertyRelative("multiplier").floatValue = speedRatio;
        appearance.FindPropertyRelative("reversed").boolValue = false;
    }

    private static void ConnectTransmissionPulleys(GameObject root, IReadOnlyDictionary<string, Transform> parts)
    {
        var rollers = root.GetComponentsInChildren<InteractionRoller>()
            .ToDictionary(roller => roller.gameObject.name);
        void Connect(string name, string drive, Transform parent, float speedRatio = 1f)
        {
            var mesh = parts["pulley_" + name];
            var roller = rollers[drive + " Physics"];
            var pivot = Create(mesh.name + " Visual Pivot", parent, parent.InverseTransformPoint(mesh.position));
            pivot.rotation = roller.transform.rotation;
            mesh.SetParent(pivot, true);
            var settings = new SerializedObject(roller);
            AddRollerModel(settings, pivot, speedRatio);
            settings.ApplyModifiedPropertiesWithoutUndo();
        }

        foreach (var side in new[] { "left", "right" })
        {
            Connect($"indexer_{side}_motor", $"indexer_{side}", root.transform);
            Connect($"indexer_{side}_wheel", $"indexer_{side}", root.transform);
            Connect($"kicker_motor_{side}", "kicker_powered", root.transform);
            Connect($"flywheel_motor_{side}", $"flywheel_{side}", root.transform);
            Connect($"backspin_intermediate_{side}_pulley", $"flywheel_{side}", root.transform);
            Connect($"backspin_intermediate_{side}_spur", $"flywheel_{side}", root.transform);
        }
        Connect("intake_driver", "intake_roller_2_slide", parts["intake_carriage"], 26f / 27f);
    }

    private static RollerGroup MakeGroup(Transform root, string name, IReadOnlyList<RollerLink> links)
    {
        var group = Create(name, root, Vector3.zero).gameObject.AddComponent<RollerGroup>();
        var properties = new SerializedObject(group);
        var members = properties.FindProperty("rollers");
        members.arraySize = links.Count;
        for (int i = 0; i < links.Count; i++)
        {
            var entry = members.GetArrayElementAtIndex(i);
            var link = links[i];
            entry.FindPropertyRelative("IntRoller").objectReferenceValue = link.Flywheel ? null : link.Roller;
            entry.FindPropertyRelative("Flywheel").objectReferenceValue = link.Flywheel;
            entry.FindPropertyRelative("Multiplier").floatValue = link.Ratio;
            entry.FindPropertyRelative("flipped").boolValue = link.Flipped;
        }
        properties.ApplyModifiedPropertiesWithoutUndo();
        return group;
    }

    private static GameObject MakeDisplayPrefab(GameObject model, Dictionary<Material, Material> converted, string template, string name)
    {
        var root = UnpackTemplate(template);
        root.name = name;
        try
        {
            var displayModel = AddModel(model, root.transform, converted);
            if (template == "BaseRobot_Spawn")
            {
                var drive = Object.Instantiate(Load<GameObject>(DriveAsset), root.transform, false);
                var wheels = drive.transform.Find("Wheels");
                wheels.localPosition = Vector3.zero;
                foreach (Transform item in wheels)
                {
                    var visual = root.GetComponentsInChildren<Transform>().Single(x => x.name.Length == 8 && x.name.StartsWith("wheel_", StringComparison.Ordinal) && Corner(x.position) == item.name);
                    item.position = visual.position;
                    item.GetComponent<WheelCollider>().radius = 0.0381f;
                }
                var settings = root.GetComponent<SpawnpointSelectionSettings>();
                var cloth = displayModel.GetComponentsInChildren<MeshRenderer>()
                    .Single(renderer => renderer.name == "bumper_cloth");
                settings.bumperMeshes = new[] { cloth };
                settings.materialIndex = 0;
                root.GetComponent<Rigidbody>().centerOfMass = Vector3.zero;
                                AddChassisDisplayCollider(root);
            }
            return SavePrefab(root, name);
        }
        finally
        {
            Object.DestroyImmediate(root);
        }
    }

    private static void AddChassisDisplayCollider(GameObject root)
    {
        AddBox("Spawn Bumper Bounds", root.transform, new Vector3(0f, 0.09525f, 0f),
            new Vector3(0.8806f, 0.1143f, 0.8806f)).tag = "Bumpers";
    }

    private static Transform AddModel(GameObject model, Transform parent, Dictionary<Material, Material> converted)
    {
        var visual = Object.Instantiate(model, parent, false);
        visual.name = "Goldfire CAD";
        visual.transform.localRotation = Quaternion.Euler(0f, 180f, 0f) * visual.transform.localRotation;
        foreach (var renderer in visual.GetComponentsInChildren<MeshRenderer>())
        {
            renderer.gameObject.layer = RobotLayer;
            renderer.sharedMaterials = renderer.sharedMaterials.Select(original => ConvertMaterial(original, converted)).ToArray();
        }
        return visual.transform;
    }

    private static Material ConvertMaterial(Material original, Dictionary<Material, Material> converted)
    {
        if (!original) return MaterialAsset("Aluminum", new Color(0.68f, 0.70f, 0.73f), 0.35f);
        if (converted.TryGetValue(original, out var material)) return material;
        if (original.name == "Goldfire_Smoked_Hopper")
        {
            material = SmokedHopperMaterial();
            converted.Add(original, material);
            return material;
        }
        if (original.name == "Goldfire_Black_Print")
        {
            material = MaterialAsset("Black 3D-Printed Mount", new Color(0.075f, 0.078f, 0.083f), 0.27f);
            material.name = "Black 3D-Printed Mount";
            converted.Add(original, material);
            return material;
        }
        if (original.name == "Goldfire_Motor_Black" || original.name == "Goldfire_Belt_Rubber")
        {
            bool belt = original.name == "Goldfire_Belt_Rubber";
            material = belt
                ? MaterialAsset("Transmission Belt Rubber", new Color(0.10f, 0.105f, 0.115f), 0.23f)
                : MaterialAsset("Black NEO Vortex Housing", new Color(0.112f, 0.123f, 0.138f), 0.43f);
            converted.Add(original, material);
            return material;
        }

        var values = original.name.Split('_');
        var fallback = original.HasProperty("_Color") ? original.color : Color.gray;
        bool parsed = values.Length == 5
            && float.TryParse(values[0], NumberStyles.Float, CultureInfo.InvariantCulture, out _)
            && float.TryParse(values[1], NumberStyles.Float, CultureInfo.InvariantCulture, out _)
            && float.TryParse(values[2], NumberStyles.Float, CultureInfo.InvariantCulture, out _);
        var color = parsed ? new Color(float.Parse(values[0], CultureInfo.InvariantCulture),
            float.Parse(values[1], CultureInfo.InvariantCulture), float.Parse(values[2], CultureInfo.InvariantCulture)) : fallback;
        material = MaterialAsset("CAD Material " + converted.Count.ToString("00"), color, 0.3f);
        converted.Add(original, material);
        return material;
    }

    private static Material MaterialAsset(string name, Color color, float smoothness)
    {
        string path = Robot + "/" + name + ".mat";
        var material = AssetDatabase.LoadAssetAtPath<Material>(path);
        if (!material)
        {
            material = new Material(Shader.Find("Universal Render Pipeline/Lit"));
            AssetDatabase.CreateAsset(material, path);
        }
        material.color = color;
        material.SetFloat("_Smoothness", smoothness);
        EditorUtility.SetDirty(material);
        return material;
    }

    private static Material SmokedHopperMaterial()
    {
        const string path = Robot + "/Smoked Hopper Panes.mat";
        var material = AssetDatabase.LoadAssetAtPath<Material>(path);
        if (!material)
        {
            material = new Material(Load<Material>("Assets/Materials/SmokedPolycarb.mat"));
            AssetDatabase.CreateAsset(material, path);
        }
        // Onshape exports the panes as opaque. This tint models the smoke shown on Goldfire.
        material.color = new Color(0.190f, 0.200f, 0.227f, 0.38f);
        material.SetFloat("_Smoothness", 0.74f);
        EditorUtility.SetDirty(material);
        return material;
    }

    private static GameObject[] MakeTutorialCards()
    {
        var template = Load<GameObject>(TutorialTemplate);
        var cards = new[]
        {
            (Name: "Intake", Title: "Slide intake", Instructions: "Hold LT to collect Fuel.\nD-pad down stows the slide."),
            (Name: "Hub", Title: "Auto shoot", Instructions: "Hold RT in your zone.\nGoldfire aims and feeds when ready."),
            (Name: "Pass", Title: "Passing", Instructions: "Hold RB to pass. RT also passes outside your alliance zone."),
        };

        return cards.Select(card =>
        {
            var instance = (GameObject)PrefabUtility.InstantiatePrefab(template);
            try
            {
                PrefabUtility.UnpackPrefabInstance(instance, PrefabUnpackMode.Completely, InteractionMode.AutomatedAction);
                instance.name = "Goldfire " + card.Title;
                var photo = instance.transform.Find("Image").GetComponent<Image>();
                photo.sprite = ImportSprite(Artwork + "/Goldfire-Card-" + card.Name + ".jpg");
                photo.preserveAspect = true;
                var label = instance.transform.Find("Text").GetComponent<TMP_Text>();
                label.text = "<b>" + card.Title + "</b>\n" + card.Instructions;
                label.enableAutoSizing = true;
                label.fontSizeMin = 18f;
                label.fontSizeMax = 25f;
                var saved = PrefabUtility.SaveAsPrefabAsset(instance, Artwork + "/Card " + card.Name + ".prefab");
                return saved ? saved : throw new InvalidOperationException("Could not save Goldfire card " + card.Name);
            }
            finally
            {
                Object.DestroyImmediate(instance);
            }
        }).ToArray();
    }

    private static Sprite ImportSprite(string path)
    {
        var settings = AssetImporter.GetAtPath(path) as TextureImporter;
        if (!settings) throw new InvalidOperationException("Missing Goldfire artwork: " + path);
        if (settings.textureType != TextureImporterType.Sprite || settings.spriteImportMode != SpriteImportMode.Single
            || settings.npotScale != TextureImporterNPOTScale.None)
        {
            settings.textureType = TextureImporterType.Sprite;
            settings.spriteImportMode = SpriteImportMode.Single;
            settings.npotScale = TextureImporterNPOTScale.None;
            settings.mipmapEnabled = false;
            settings.SaveAndReimport();
        }
        return Load<Sprite>(path);
    }

    private static void WriteWorkshopDraft(RebuiltModpackSO pack)
    {
        var draft = ModpackWorkshopMetadataEditorUtility.FindMetadata(pack) ??
            ModpackWorkshopMetadataEditorUtility.CreateMetadata(pack);
        draft.PreviewImage = Load<Texture2D>(HeroImage);
        EditorUtility.SetDirty(draft);
    }

    private static void ValidateReleaseAssets(RebuiltModpackSO pack, bool checkWorkshopDraft = true)
    {
        if (!pack.Thumbnail) throw new InvalidOperationException("Goldfire mod browser artwork is missing.");
        var robot = pack.Robots.Single();
        if (!robot.RobotImage || robot.RobotImage.name != "Awtybots-Robot-Selection" ||
            !robot.MainMenuPrefab.GetComponent<Animator>() || robot.TutorialCards.Count != 3)
            throw new InvalidOperationException("Goldfire menu, artwork, or tutorial card references are incomplete.");
        foreach (var card in robot.TutorialCards)
        {
            if (!card.transform.Find("Image").GetComponent<Image>().sprite ||
                string.IsNullOrWhiteSpace(card.GetComponentInChildren<TMP_Text>().text))
                throw new InvalidOperationException("Goldfire tutorial card lacks artwork or instructions.");
        }

        var spawn = robot.SpawnSelectionPrefab;
        var body = spawn.GetComponent<Rigidbody>();
        var settings = spawn.GetComponent<SpawnpointSelectionSettings>();
        if (Mathf.Abs(body.mass - 98f) > 0.01f || Mathf.Abs(body.drag - 4f) > 0.01f
            || Mathf.Abs(body.angularDrag - 5f) > 0.01f || body.centerOfMass != Vector3.zero
            || settings.bumperMeshes.Length != 1 || settings.materialIndex != 0
            || !spawn.GetComponentsInChildren<Collider>().Any(collider => collider.CompareTag("Bumpers")))
            throw new InvalidOperationException("Goldfire spawn prefab does not match the documented bumper and Rigidbody setup.");

        var renderers = robot.RobotPrefab.GetComponentsInChildren<MeshRenderer>();
        var belts = renderers.Where(renderer => renderer.name.StartsWith("belt_", StringComparison.Ordinal)).ToArray();
        var intakeCarriage = robot.RobotPrefab.transform.Find("Intake Rails/Intake Carriage");
        if (belts.Length != 16 || !intakeCarriage ||
            belts.Count(belt => belt.transform.IsChildOf(intakeCarriage)) != 2 ||
            belts.Any(belt => !belt.GetComponent<MeshFilter>().sharedMesh || belt.GetComponent<Collider>()))
            throw new InvalidOperationException("Goldfire belt visuals are missing or attached to the wrong assembly.");

        var pulleyModels = robot.RobotPrefab.GetComponentsInChildren<InteractionRoller>()
            .SelectMany(roller =>
            {
                var models = new SerializedObject(roller).FindProperty("models");
                return Enumerable.Range(0, models.arraySize)
                    .Select(index => models.GetArrayElementAtIndex(index).FindPropertyRelative("model")
                        .objectReferenceValue as GameObject).ToArray();
            }).Where(pivot => pivot && pivot.name.StartsWith("pulley_", StringComparison.Ordinal)).ToArray();
        if (pulleyModels.Length != 13 || pulleyModels.Distinct().Count() != 13 ||
            pulleyModels.Count(pivot => pivot.name == "pulley_intake_driver Visual Pivot" &&
                pivot.transform.IsChildOf(intakeCarriage)) != 1 ||
            robot.RobotPrefab.GetComponentsInChildren<AwtybotsV1WheelVisual>().Length != 4)
            throw new InvalidOperationException("The Goldfire pulleys or wheel visuals lack their rotating pivots.");

        var panes = renderers
            .Where(renderer => renderer.name.Contains("_smoked_pane_", StringComparison.Ordinal)).ToArray();
        if (panes.Length != 17 ||
            panes.Count(pane => pane.name == "chassis_spindexer_smoked_pane_01") != 1 ||
            panes.Count(pane => pane.name == "chassis_bottom_smoked_pane_01") != 1 ||
            panes.Count(pane => pane.name == "chassis_spindexer_support_smoked_pane_01") != 1 ||
            panes.Any(pane => pane.sharedMaterials.Length != 1 ||
            pane.sharedMaterial != panes[0].sharedMaterial ||
            pane.sharedMaterial.GetTag("RenderType", false) != "Transparent" ||
            Mathf.Abs(pane.sharedMaterial.color.a - 0.38f) > 0.01f))
            throw new InvalidOperationException("Goldfire pane meshes did not retain distinct transparent finishes.");

        foreach (var side in new[] { "left", "right" })
        {
            string axleName = $"indexer_{side}";
            var wheel = renderers.Single(renderer => renderer.name == axleName + "_omni_wheel_01");
            var wheelMesh = wheel.GetComponent<MeshFilter>().sharedMesh;
            var cone = renderers.Single(renderer => renderer.name == axleName + "_mesh");
            var axle = wheel.GetComponentInParent<InteractionRoller>(true);
            if (!wheelMesh || wheelMesh.vertexCount < 1000 || wheel.sharedMaterials.Length != 2)
                throw new InvalidOperationException($"Goldfire {side} spindexer must retain the rim and roller materials.");
            if (!axle || axle.gameObject.name != axleName + " Physics" ||
                wheel.transform.parent != cone.transform.parent)
                throw new InvalidOperationException($"Goldfire {side} spindexer must retain its omni wheel on the cone axle.");
        }

        var housings = renderers.Where(renderer => renderer.name.Contains("_controller_housing_", StringComparison.Ordinal)).ToArray();
        int shooterHousings = housings.Count(renderer => renderer.name.Contains("_shooter_controller_housing_", StringComparison.Ordinal));
        int driveHousings = housings.Count(renderer => renderer.name.Contains("_drive_controller_housing_", StringComparison.Ordinal));
        if (housings.Length != 15 || shooterHousings != 6 || driveHousings != 4 ||
            housings.Any(renderer => renderer.GetComponent<MeshFilter>().sharedMesh.vertexCount == 0))
            throw new InvalidOperationException("Goldfire Vortex controller housings are missing.");

        if (housings.Where(renderer => renderer.name.Contains("_shooter_controller_housing_", StringComparison.Ordinal))
            .Any(renderer => !renderer.sharedMaterials.Any(material => material.color.r > 0.65f && material.color.g < 0.15f)))
            throw new InvalidOperationException("Goldfire shooter housings lost the CAD power terminal materials.");

        var mounts = renderers.Where(renderer => renderer.name.Contains("_black_print_mount_", StringComparison.Ordinal)).ToArray();
        if (mounts.Length != 2 || mounts.Any(renderer => renderer.sharedMaterials.Length != 1 ||
            renderer.sharedMaterial.name != "Black 3D-Printed Mount" ||
            renderer.sharedMaterial.color.a != 1f))
            throw new InvalidOperationException("Goldfire climber mounts did not retain the black print material.");

        var taggers = robot.RobotPrefab.GetComponentsInChildren<FlywheelPenaltyTagger>();
        if (taggers.Length != 2 || taggers.Any(tagger =>
            new SerializedObject(tagger).FindProperty("gamePieceLayers").intValue != FuelLayer))
            throw new InvalidOperationException("Both Goldfire launcher rollers must tag REBUILT Fuel.");

        if (checkWorkshopDraft)
        {
            var draft = ModpackWorkshopMetadataEditorUtility.FindMetadata(pack);
            if (draft == null || !draft.PreviewImage || draft.PreviewImage.width * 9 != draft.PreviewImage.height * 16 ||
                new FileInfo(AssetDatabase.GetAssetPath(draft.PreviewImage)).Length >= 1_000_000)
                throw new InvalidOperationException("Goldfire Workshop preview must be 16:9 and under 1 MB.");
        }
        Debug.Log($"GOLDFIRE RELEASE: {panes.Length} transparent panes, 2 spindexer omni wheels, 16 belts, 13 driven pulley/gear pivots, and 4 rolling swerve visuals passed. Cards, bumpers, Vortex housings, and 2 Fuel penalty taggers passed.");
    }

    public static void ValidateWindowsBundle()
    {
        string path = Path.GetFullPath("BuiltMods/rebuilt_awtybots_rebuilt/standalonewindows64/rebuilt_awtybots_rebuilt_assets");
        var bundle = AssetBundle.LoadFromFile(path);
        if (!bundle) throw new InvalidOperationException("The Goldfire Windows bundle could not be opened.");
        try
        {
            ValidateReleaseAssets(bundle.LoadAllAssets<RebuiltModpackSO>().Single(), checkWorkshopDraft: false);
            Debug.Log("GOLDFIRE RELEASE: Verified the Windows bundle references.");
        }
        finally
        {
            bundle.Unload(true);
        }
    }

    private static RebuiltRobotMetadataSO WriteRobotMetadata(GameObject match, GameObject menu, GameObject spawn, IReadOnlyList<GameObject> cards)
    {
        string path = Robot + "/GoldfireV1.asset";
        var data = AssetDatabase.LoadAssetAtPath<RebuiltRobotMetadataSO>(path);
        if (!data) data = ScriptableObject.CreateInstance<RebuiltRobotMetadataSO>();
        var properties = new SerializedObject(data);
        properties.FindProperty("teamNumber").intValue = 5829;
        properties.FindProperty("teamName").stringValue = "Awtybots";
        SetReference(properties, "robotImage", ImportSprite(RobotSelectionImage));
        var main = properties.FindProperty("mainRobot");
        main.FindPropertyRelative("displayName").stringValue = "Goldfire V1";
        main.FindPropertyRelative("robotPrefab").objectReferenceValue = match;
        main.FindPropertyRelative("mainMenuPrefab").objectReferenceValue = menu;
        main.FindPropertyRelative("spawnSelectionPrefab").objectReferenceValue = spawn;
        var cardList = main.FindPropertyRelative("tutorialCards");
        cardList.arraySize = cards.Count;
        for (int i = 0; i < cards.Count; i++)
            cardList.GetArrayElementAtIndex(i).objectReferenceValue = cards[i];
        main.FindPropertyRelative("preloadGamePiece").objectReferenceValue =
            Load<GameObject>("Assets/Prefabs/Games/Rebuilt/Fuel.prefab");
        properties.ApplyModifiedPropertiesWithoutUndo();
        if (!AssetDatabase.Contains(data)) AssetDatabase.CreateAsset(data, path);
        EditorUtility.SetDirty(data);
        return data;
    }

    private static RebuiltModpackSO WritePack(RebuiltRobotMetadataSO robot)
    {
        string path = Pack + "/AwtybotsRebuilt.asset";
        var data = AssetDatabase.LoadAssetAtPath<RebuiltModpackSO>(path);
        if (!data) data = ScriptableObject.CreateInstance<RebuiltModpackSO>();
        var properties = new SerializedObject(data);
        properties.FindProperty("modpackName").stringValue = "Awtybots REBUILT";
        properties.FindProperty("modpackVersion").stringValue = "0.1.15";
        properties.FindProperty("authorName").stringValue = "Awtybots";
        properties.FindProperty("description").stringValue = "Team 5829 Goldfire V1 for REBUILT.";
        SetReference(properties, "thumbnail", ImportSprite(HeroImage));
        var robots = properties.FindProperty("robots");
        robots.arraySize = 1;
        robots.GetArrayElementAtIndex(0).objectReferenceValue = robot;
        properties.ApplyModifiedPropertiesWithoutUndo();
        if (!AssetDatabase.Contains(data)) AssetDatabase.CreateAsset(data, path);
        EditorUtility.SetDirty(data);
        return data;
    }

    private static string Corner(Vector3 position)
    {
        return (position.z >= 0f ? "F" : "B") + (position.x >= 0f ? "R" : "L");
    }

    private static Transform Create(string name, Transform parent, Vector3 localPosition)
    {
        var child = new GameObject(name) { layer = RobotLayer };
        child.transform.SetParent(parent, false);
        child.transform.localPosition = localPosition;
        return child.transform;
    }

    private static BoxCollider AddBox(string name, Transform parent, Vector3 worldPosition, Vector3 size)
    {
        var collider = Create(name, parent, parent.InverseTransformPoint(worldPosition)).gameObject.AddComponent<BoxCollider>();
        collider.size = size;
        return collider;
    }

    private static GameObject UnpackTemplate(string name)
    {
        var root = (GameObject)PrefabUtility.InstantiatePrefab(Load<GameObject>(Template + name + ".prefab"));
        PrefabUtility.UnpackPrefabInstance(root, PrefabUnpackMode.Completely, InteractionMode.AutomatedAction);
        return root;
    }

    private static T Load<T>(string path) where T : Object
    {
        var asset = AssetDatabase.LoadAssetAtPath<T>(path);
        return asset ? asset : throw new InvalidOperationException("Missing asset " + path);
    }

    private static GameObject SavePrefab(GameObject root, string name)
    {
        var prefab = PrefabUtility.SaveAsPrefabAsset(root, Robot + "/" + name + ".prefab");
        return prefab ? prefab : throw new InvalidOperationException("Could not save " + name);
    }

    private static void SetReference(SerializedObject target, string property, Object value)
    {
        target.FindProperty(property).objectReferenceValue = value;
    }

    private static void SetFloat(SerializedObject target, string property, float value)
    {
        target.FindProperty(property).floatValue = value;
    }
}

