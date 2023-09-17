namespace FluidDynamics.Models
{
    public class FluidCube
    {
        public int size;
        public float deltaTime;
        public float diffusion;
        public float viscosity;
        public float[] pDensities;
        public float[] densities;
        
        public float[] Vx;
        public float[] Vy;
        public float[] Vz;
        
        public float[] Vx0;
        public float[] Vy0;
        public float[] Vz0;


        public FluidCube(int size, int diffusion, int viscosity, float deltaTime)
        {
            int n = size;
            this.size = size;
            this.deltaTime = deltaTime;
            this.diffusion = diffusion;
            this.viscosity = viscosity;

            pDensities = new float[n * n * n];
            densities = new float[n * n * n];

            Vx = new float[n * n * n];
            Vy = new float[n * n * n];
            Vz = new float[n * n * n];

            Vx0 = new float[n * n * n];
            Vy0 = new float[n * n * n];
            Vz0 = new float[n * n * n];

        }
    }
}