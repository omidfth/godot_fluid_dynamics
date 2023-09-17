using System.Collections.Generic;
using System.Threading.Tasks;
using FluidDynamics.Models;
using Godot;

namespace FluidDynamics;

public partial class FluidView : Node
{
	public Color color = Colors.White;
	public PackedScene pixel;
	public int size = 16;
	public int diffusion;
	public int viscosity;
	public int iteration = 1;
	public float deltaTime = 0.01f;
	public Vector3 velocityPower = new Vector3(0, 1000, 0);
	public float densityPower = 5;

	private int _dt;
	private List<MeshInstance3D> _pixels = new List<MeshInstance3D>();

	private FluidCube _cube;
	private Simulator _simulator;
	private bool _isReady;


	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		GD.Print("Fluid View Starting ...");
		pixel = (PackedScene)ResourceLoader.Load("res://mesh_instance_3d.tscn");
		MeshInstance3D meshIns = (MeshInstance3D)pixel.Instantiate();
		_dt = (int)(deltaTime * 1000);
		_cube = new FluidCube(size, diffusion, viscosity, deltaTime);
		GD.Print("Size: " + size);
		for (int i = -size / 2; i < size / 2; i++)
		{
			for (int j = -size / 2; j < size / 2; j++)
			{
				for (int k = -size / 2; k < size / 2; k++)
				{
					// Create a new MeshInstance node.
					var meshInstance = new MeshInstance3D();

					// Set the mesh of the MeshInstance node.
					meshInstance.Mesh = meshIns.Mesh;
					var material = new StandardMaterial3D();
					material.AlbedoColor = color;
					meshInstance.MaterialOverride = material;
					AddChild(meshInstance);
					// Set the position of the MeshInstance node.
					meshInstance.Position = new Vector3(i, j, k);
GD.Print(meshInstance.Position);
					_pixels.Add(meshInstance);
				}
			}
		}

		ChangeColor();
		_simulator = new Simulator(iteration, _cube);
		_isReady = true;
		UpdateSimulate();
	}

	void AddVelocity()
	{
		_simulator.AddVelocity(8, 2, 8, velocityPower.X, velocityPower.Y, velocityPower.Z);
	}

	void AddDensity()
	{
		_simulator.AddDensity(8, 2, 8, densityPower);
	}

	async void ChangeColor()
	{
		for (int i = 0; i < _pixels.Count; i++)
		{
			var mat = (StandardMaterial3D)_pixels[i].MaterialOverride;
			mat.Transparency = BaseMaterial3D.TransparencyEnum.Alpha;

			mat.AlbedoColor =
				new Color(0, 0, 0, _cube.densities[i]);
			_pixels[i].MaterialOverride = mat;
		}
	}


	async void UpdateSimulate()
	{
		while (_isReady)
		{
			if (Input.IsMouseButtonPressed(MouseButton.Left))
			{
				AddDensity();
				AddVelocity();
			}

			_simulator.Step();
			ChangeColor();
			await Task.Delay(20);
		}
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
		GD.Print(Engine.GetFramesPerSecond());
	}
}
