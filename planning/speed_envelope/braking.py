"""Scalar braking bound only; does not certify swept-volume clearance."""
import math


def _validate(distance, delay, deceleration, margin):
    if not all(math.isfinite(v) for v in (distance, delay, deceleration, margin)):
        raise ValueError("braking inputs must be finite")
    if min(distance, delay, margin) < 0 or deceleration <= 0:
        raise ValueError("invalid braking bounds")


def stopping_distance(speed, delay, deceleration, margin):
    _validate(speed, delay, deceleration, margin)
    result = speed * delay + speed * speed / (2 * deceleration) + margin
    if not math.isfinite(result):
        raise ValueError("braking calculation exceeds numerical range")
    return result


def speed_limit(free_distance, delay, deceleration, margin):
    _validate(free_distance, delay, deceleration, margin)
    available = max(0.0, free_distance - margin)
    if available == 0.:
        return 0.
    # Rationalized root avoids cancellation at small distances / long delays.
    root = math.hypot(deceleration * delay, math.sqrt(2 * deceleration * available))
    denominator = root + deceleration * delay
    result = 0.0 if denominator == 0 else 2 * deceleration * available / denominator
    if not math.isfinite(result):
        raise ValueError("braking calculation exceeds numerical range")
    return result
