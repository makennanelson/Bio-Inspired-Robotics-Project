"""Show one of the existing prescribed gait simulations in the MuJoCo viewer."""
import argparse

from gait_sim import GAITS, SpringParameters, run_simulation


def main():
    parser = argparse.ArgumentParser(description="View the simulated biped walking or running.")
    parser.add_argument("--gait", choices=("walk", "run"), default="walk")
    parser.add_argument("--kappa", type=float, default=15.0,
                        help="dimensionless spring stiffness (default: 15)")
    parser.add_argument("--zeta", type=float, default=0.20,
                        help="spring damping ratio (default: 0.20)")
    parser.add_argument("--ground-kappa", type=float, default=100.0,
                        help="dimensionless ground stiffness (default: 100)")
    args = parser.parse_args()

    params = SpringParameters(
        dimensionless_stiffness=args.kappa,
        damping_ratio=args.zeta,
        dimensionless_ground_stiffness=args.ground_kappa,
    )
    print(f"Opening {args.gait} simulation: leg kappa={args.kappa:g}, "
          f"ground kappa={args.ground_kappa:g}, zeta={args.zeta:g}")
    result = run_simulation(GAITS[args.gait], params, visualize=True)
    print("Simulation summary after closing the viewer:")
    print(f"  stable: {result['stable']}")
    print(f"  cost of transport: {result['cost_of_transport']:.3f}")
    print(f"  distance: {result['distance_m']:.2f} m")
    print(f"  actual flight fraction: {result['flight_fraction']:.3f}")
    print(f"  scheduled flight fraction: {result['scheduled_flight_fraction']:.3f}")


if __name__ == "__main__":
    main()
