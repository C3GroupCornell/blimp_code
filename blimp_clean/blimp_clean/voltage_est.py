
import numpy as np

# Paste your measured data here (same length for both lists).
VOLTAGES = [0.3,0.8]
GRAMS_OUTPUT = [1,3]

GRAM_TO_NEWTON = 0.00980665


def _validate_inputs(voltages: np.ndarray, grams_output: np.ndarray) -> None:
    if voltages.size == 0 or grams_output.size == 0:
        raise ValueError("Fill VOLTAGES and GRAMS_OUTPUT with at least one sample.")
    if voltages.size != grams_output.size:
        raise ValueError("VOLTAGES and GRAMS_OUTPUT must have the same number of values.")


def estimate_voltage_constant(
    voltages: np.ndarray,
    grams_output: np.ndarray,
    through_origin: bool = True,
) -> dict:
    """Fit grams_output ~= k * voltage (+ b if through_origin=False)."""
    _validate_inputs(voltages, grams_output)

    x = np.asarray(voltages, dtype=float)
    y = np.asarray(grams_output, dtype=float)

    if through_origin:
        # Least-squares solution for y = kx.
        denom = float(np.dot(x, x))
        if denom == 0.0:
            raise ValueError("All voltages are zero; cannot estimate a voltage constant.")
        k_grams_per_volt = float(np.dot(x, y) / denom)
        intercept_grams = 0.0
        y_hat = k_grams_per_volt * x
    else:
        # Least-squares solution for y = kx + b.
        A = np.column_stack((x, np.ones_like(x)))
        k_grams_per_volt, intercept_grams = np.linalg.lstsq(A, y, rcond=None)[0]
        y_hat = k_grams_per_volt * x + intercept_grams

    residuals = y - y_hat
    ss_res = float(np.dot(residuals, residuals))
    ss_tot = float(np.dot(y - y.mean(), y - y.mean()))
    r_squared = 1.0 if ss_tot == 0.0 else 1.0 - (ss_res / ss_tot)

    return {
        "k_grams_per_volt": k_grams_per_volt,
        "k_newton_per_volt": k_grams_per_volt * GRAM_TO_NEWTON,
        "intercept_grams": float(intercept_grams),
        "r_squared": r_squared,
        "rmse_grams": float(np.sqrt(np.mean(residuals**2))),
    }


if __name__ == "__main__":
    v = np.array(VOLTAGES, dtype=float)
    g = np.array(GRAMS_OUTPUT, dtype=float)

    # Set to False if your data has a non-zero offset at 0 V.
    result = estimate_voltage_constant(v, g, through_origin=True)

    print(f"Voltage constant k: {result['k_grams_per_volt']:.6f} g/V")
    print(f"Voltage constant k: {result['k_newton_per_volt']:.6f} N/V")
    print(f"Intercept:          {result['intercept_grams']:.6f} g")
    print(f"R^2:                {result['r_squared']:.6f}")
    print(f"RMSE:               {result['rmse_grams']:.6f} g")
