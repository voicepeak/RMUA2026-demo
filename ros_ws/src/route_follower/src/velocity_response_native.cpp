// Exact counterpart of VelocityResponse's fixed-step stopping integration.
// Geometry certification remains in Python against the latest measured cloud.
#include <algorithm>
#include <cmath>
#include <vector>

namespace {
double clip(double value, double lo, double hi) {
    return std::max(lo, std::min(hi, value));
}

double interpolate(const double* path, const std::vector<double>& arc,
                   double station, int axis) {
    const int count = static_cast<int>(arc.size());
    if (station < arc.front()) return path[axis];
    if (station >= arc.back()) return path[(count - 1) * 3 + axis];
    const int high = std::upper_bound(arc.begin(), arc.end(), station) - arc.begin();
    const int low = high - 1;
    const double fraction = (station - arc[low]) / (arc[high] - arc[low]);
    return path[low * 3 + axis] + fraction * (path[high * 3 + axis] - path[low * 3 + axis]);
}

double vertical_target(const double* path, const std::vector<double>& arc,
                       const double* position, const double* velocity) {
    if (arc.empty()) return 0.;
    double best = INFINITY, fraction_best = 0.;
    int nearest = -1;
    for (int i = 0; i + 1 < static_cast<int>(arc.size()); ++i) {
        const double dx = path[(i + 1) * 3] - path[i * 3];
        const double dy = path[(i + 1) * 3 + 1] - path[i * 3 + 1];
        const double length = dx * dx + dy * dy;
        if (length < 1e-8) continue;
        const double px = position[0] - path[i * 3];
        const double py = position[1] - path[i * 3 + 1];
        const double fraction = clip((px * dx + py * dy) / std::max(1e-8, length), 0., 1.);
        const double rx = px - fraction * dx, ry = py - fraction * dy;
        const double distance = rx * rx + ry * ry;
        if (distance < best) {
            best = distance; nearest = i; fraction_best = fraction;
        }
    }
    if (nearest < 0) return clip(path[(arc.size() - 1) * 3 + 2] - position[2], -4., 4.5);
    const double reference = path[nearest * 3 + 2] + fraction_best *
        (path[(nearest + 1) * 3 + 2] - path[nearest * 3 + 2]);
    const double along = arc[nearest] + fraction_best * (arc[nearest + 1] - arc[nearest]);
    const double lo = std::max(0., along - 2.), hi = std::min(arc.back(), along + 2.);
    const double dx = interpolate(path, arc, hi, 0) - interpolate(path, arc, lo, 0);
    const double dy = interpolate(path, arc, hi, 1) - interpolate(path, arc, lo, 1);
    const double length = std::max(1e-8, std::sqrt(dx * dx + dy * dy));
    const double slope = (interpolate(path, arc, hi, 2) - interpolate(path, arc, lo, 2)) /
        std::max(1e-8, hi - lo);
    return clip((velocity[0] * dx + velocity[1] * dy) / length * slope + reference - position[2], -4., 4.5);
}
}

extern "C" int rmua_response_abi() { return 2; }

extern "C" int rmua_response_integrate(
    const double* position, const double* velocity, const double* commands,
    int command_count, const double* scenarios, int scenario_count,
    const double* path, int path_count, double latency, double hold,
    double deceleration, double feedback, double lift_gain, double command_max, double duration,
    int capacity, double* output, double* times) {
    if (command_count < 1 || scenario_count < 1 || capacity < 3 || deceleration <= 0.) return -1;
    const int states = command_count * scenario_count;
    std::vector<double> p(states * 3), v(states * 3), u(states * 3), arc(path_count);
    for (int i = 1; i < path_count; ++i) {
        const double dx = path[i * 3] - path[(i - 1) * 3];
        const double dy = path[i * 3 + 1] - path[(i - 1) * 3 + 1];
        arc[i] = arc[i - 1] + std::sqrt(dx * dx + dy * dy);
    }
    for (int state = 0; state < states; ++state) {
        for (int axis = 0; axis < 3; ++axis) {
            const int offset = state * 3 + axis;
            u[offset] = commands[(state / scenario_count) * 3 + axis];
            v[offset] = velocity[axis];
            p[offset] = position[axis] + latency * velocity[axis];
            output[offset] = position[axis];
            output[states * 3 + offset] = p[offset];
        }
    }
    times[0] = 0.; times[1] = latency;
    const double step = .08;
    double elapsed = 0.;
    int count = 2;
    while (elapsed < duration) {
        if (count >= capacity) return -2;
        double max_u = 0., max_v = 0.;
        for (int state = 0; state < states; ++state) {
            const int offset = state * 3, scenario = state % scenario_count;
            double* ps = p.data() + offset;
            double* vs = v.data() + offset;
            double* us = u.data() + offset;
            if (elapsed >= hold) {
                double tx = -feedback * vs[0], ty = -feedback * vs[1];
                const double scale = std::min(1., 3. / std::max(1e-9, std::sqrt(tx * tx + ty * ty)));
                tx *= scale; ty *= scale;
                const double dx = tx - us[0], dy = ty - us[1];
                const double fraction = std::min(1., deceleration * step /
                    std::max(1e-9, std::sqrt(dx * dx + dy * dy)));
                us[0] += fraction * dx; us[1] += fraction * dy;
                const double ex = us[0] - vs[0], ey = us[1] - vs[1];
                us[2] = clip(vertical_target(path, arc, ps, vs) + lift_gain * (ex * ex + ey * ey), -4., command_max);
            }
            const double ex = us[0] - vs[0], ey = us[1] - vs[1];
            const double effective[3] = {us[0], us[1], us[2] - scenarios[scenario * 3 + 2] * (ex * ex + ey * ey)};
            for (int axis = 0; axis < 3; ++axis) {
                const double tau = scenarios[scenario * 3 + (axis == 2 ? 1 : 0)];
                const double decay = std::exp(-step / tau);
                ps[axis] += effective[axis] * step + (vs[axis] - effective[axis]) * tau * (1. - decay);
                vs[axis] = effective[axis] + (vs[axis] - effective[axis]) * decay;
                output[(count * states + state) * 3 + axis] = ps[axis];
            }
            max_u = std::max(max_u, std::sqrt(us[0] * us[0] + us[1] * us[1]));
            max_v = std::max(max_v, std::sqrt(vs[0] * vs[0] + vs[1] * vs[1] + vs[2] * vs[2]));
        }
        elapsed += step; times[count++] = latency + elapsed;
        if (elapsed > hold + 1. && max_u < .001 && max_v < .01) break;
    }
    return count;
}
