// Exact counterpart of VelocityResponse's fixed-step stopping integration.
// Geometry certification remains in Python against the latest measured cloud.
#include <algorithm>
#include <cmath>
#include <cstdint>
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
                       const double* position, const double* velocity, double height_gain) {
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
    if (nearest < 0) return clip(height_gain * (path[(arc.size() - 1) * 3 + 2] - position[2]), -4., 4.5);
    const double reference = path[nearest * 3 + 2] + fraction_best *
        (path[(nearest + 1) * 3 + 2] - path[nearest * 3 + 2]);
    const double along = arc[nearest] + fraction_best * (arc[nearest + 1] - arc[nearest]);
    const double lo = std::max(0., along - 2.), hi = std::min(arc.back(), along + 2.);
    const double dx = interpolate(path, arc, hi, 0) - interpolate(path, arc, lo, 0);
    const double dy = interpolate(path, arc, hi, 1) - interpolate(path, arc, lo, 1);
    const double length = std::max(1e-8, std::sqrt(dx * dx + dy * dy));
    const double slope = (interpolate(path, arc, hi, 2) - interpolate(path, arc, lo, 2)) /
        std::max(1e-8, hi - lo);
    return clip((velocity[0] * dx + velocity[1] * dy) / length * slope + height_gain * (reference - position[2]), -4., 4.5);
}

void braking_target(const double* path, int path_count, const double* position,
                    const double* velocity, double feedback, double& tx, double& ty) {
    tx = -feedback * velocity[0]; ty = -feedback * velocity[1];
    const double scale = std::min(1., 3. / std::max(1e-9, std::hypot(tx, ty)));
    tx *= scale; ty *= scale;
    const double speed = std::hypot(velocity[0], velocity[1]);
    const double gx = position[0] + .8 * velocity[0], gy = position[1] + .8 * velocity[1];
    double best = INFINITY, ex = 0., ey = 0.;
    for (int i = 0; i + 1 < path_count; ++i) {
        const double dx = path[(i + 1) * 3] - path[i * 3];
        const double dy = path[(i + 1) * 3 + 1] - path[i * 3 + 1];
        const double length = dx * dx + dy * dy;
        if (length < 1e-8) continue;
        const double px = gx - path[i * 3], py = gy - path[i * 3 + 1];
        const double fraction = clip((px * dx + py * dy) / length, 0., 1.);
        const double rx = px - fraction * dx, ry = py - fraction * dy;
        const double distance = rx * rx + ry * ry;
        if (distance < best) {best = distance; ex = rx; ey = ry;}
    }
    if (!std::isfinite(best)) return;
    const double nx = velocity[0] / std::max(1e-8, speed);
    const double ny = velocity[1] / std::max(1e-8, speed);
    const double along = ex * nx + ey * ny;
    ex -= along * nx; ey -= along * ny;
    double cx = -3. * ex, cy = -3. * ey;
    const double correction_scale = std::min(1., 8. / std::max(1e-8, std::hypot(cx, cy))) * std::min(1., speed);
    tx += cx * correction_scale; ty += cy * correction_scale;
}
}

extern "C" int rmua_response_abi() { return 12; }

// Same segment order, fractions and endpoint as swept_samples(), with timed
// samples grouped by candidate then scenario. Null outputs count allocation.
extern "C" int64_t rmua_response_samples(const double* path, const double* times,
    int point_count, int candidate_count, int scenario_count, double spacing,
    const double* origin, double* queries, double* query_times,
    int64_t* offsets, double* extents) {
    if (point_count < 2 || spacing <= 0.) return -1;
    const int states = candidate_count * scenario_count;
    int64_t cursor = 0;
    for (int candidate = 0; candidate < candidate_count; ++candidate) {
        offsets[candidate] = cursor;
        double maximum_squared = 0.;
        for (int scenario = 0; scenario < scenario_count; ++scenario) {
            const int state = candidate * scenario_count + scenario;
            for (int segment = 0; segment + 1 < point_count; ++segment) {
                const double* start = path + (segment * states + state) * 3;
                const double* finish = start + states * 3;
                const double dx = finish[0] - start[0], dy = finish[1] - start[1], dz = finish[2] - start[2];
                const int count = std::max(1, static_cast<int>(std::ceil(std::sqrt(dx * dx + dy * dy + dz * dz) / spacing)));
                if (!queries) {cursor += count; continue;}
                for (int sample = 0; sample < count; ++sample) {
                    const double fraction = static_cast<double>(sample) / count;
                    const double value[3] = {start[0] + fraction * dx, start[1] + fraction * dy, start[2] + fraction * dz};
                    double squared = 0.;
                    for (int axis = 0; axis < 3; ++axis) {
                        queries[cursor * 3 + axis] = value[axis];
                        squared += (value[axis] - origin[axis]) * (value[axis] - origin[axis]);
                    }
                    maximum_squared = std::max(maximum_squared, squared);
                    query_times[cursor++] = times[segment] + fraction * (times[segment + 1] - times[segment]);
                }
            }
            if (queries) {
                const double* end = path + ((point_count - 1) * states + state) * 3;
                double squared = 0.;
                for (int axis = 0; axis < 3; ++axis) {
                    queries[cursor * 3 + axis] = end[axis];
                    squared += (end[axis] - origin[axis]) * (end[axis] - origin[axis]);
                }
                maximum_squared = std::max(maximum_squared, squared);
                query_times[cursor] = times[point_count - 1];
            }
            ++cursor;
        }
        if (extents) extents[candidate] = std::sqrt(maximum_squared);
    }
    offsets[candidate_count] = cursor;
    return cursor;
}

extern "C" void rmua_roof_queries(const double* queries, int query_count,
    const double* origin, const double* coefficients, const double* hull,
    const double* normals, int hull_count, double road_height, int known_height,
    double margin, double* distances, double* floors) {
    const double norm = std::sqrt(1. + (coefficients[0] * coefficients[0] + coefficients[1] * coefficients[1]));
    for (int q = 0; q < query_count; ++q) {
        distances[q] = INFINITY; floors[q] = NAN;
        const double x = queries[q * 3] - origin[0], y = queries[q * 3 + 1] - origin[1];
        bool inside = true;
        for (int i = 0; i < hull_count; ++i) {
            if ((x - hull[i * 2]) * normals[i * 2] + (y - hull[i * 2 + 1]) * normals[i * 2 + 1] < 0.) {
                inside = false; break;
            }
        }
        if (!inside) continue;
        const double roof_xy = x * coefficients[0] + y * coefficients[1];
        const double height = (queries[q * 3 + 2] - origin[2] - roof_xy) - coefficients[2];
        distances[q] = (known_height ? std::min(height, road_height - height) : std::abs(height)) / norm;
        if (known_height) floors[q] = ((origin[2] + roof_xy) + coefficients[2]) + road_height - margin * norm;
    }
}

extern "C" void rmua_patch_distances(const double* queries, int query_count,
    const int64_t* ids, int neighbor_count, const double* centers,
    const double* normals, const double* bases, const double* boundaries,
    const double* offsets, int patch_count, double* distances) {
    for (int q = 0; q < query_count; ++q) {
        double best = INFINITY;
        for (int neighbor = 0; neighbor < neighbor_count; ++neighbor) {
            const int64_t i = ids[q * neighbor_count + neighbor];
            if (i < 0 || i >= patch_count) continue;
            const double dx = queries[q * 3] - centers[i * 3];
            const double dy = queries[q * 3 + 1] - centers[i * 3 + 1];
            const double dz = queries[q * 3 + 2] - centers[i * 3 + 2];
            const double x = dx * bases[i * 6] + dy * bases[i * 6 + 2] + dz * bases[i * 6 + 4];
            const double y = dx * bases[i * 6 + 1] + dy * bases[i * 6 + 3] + dz * bases[i * 6 + 5];
            bool inside = true;
            for (int edge = 0; edge < 16; ++edge) {
                if (!std::isfinite(offsets[i * 16 + edge])) break;
                if (x * boundaries[i * 32 + edge * 2] + y * boundaries[i * 32 + edge * 2 + 1] < offsets[i * 16 + edge] - 1e-7) {
                    inside = false; break;
                }
            }
            if (inside) best = std::min(best, std::abs(dx * normals[i * 3] + dy * normals[i * 3 + 1] + dz * normals[i * 3 + 2]));
        }
        distances[q] = best;
    }
}

extern "C" void rmua_route_stations(const double* queries, int query_count,
    const double* stations, const double* road, const double* segments,
    const double* lengths, int segment_count, double* output) {
    bool has_valid = false;
    for (int i = 0; i < segment_count; ++i) has_valid |= lengths[i] > 1e-8;
    for (int q = 0; q < query_count; ++q) {
        int nearest = 0;
        double best = INFINITY, selected_fraction = 0.;
        for (int i = 0; i < segment_count; ++i) {
            if (has_valid && lengths[i] <= 1e-8) continue;
            double dx = queries[q * 3] - road[i * 2];
            double dy = queries[q * 3 + 1] - road[i * 2 + 1];
            const double fraction = clip((dx * segments[i * 2] + dy * segments[i * 2 + 1]) /
                std::max(1e-8, lengths[i]), 0., 1.);
            dx -= fraction * segments[i * 2]; dy -= fraction * segments[i * 2 + 1];
            const double distance = dx * dx + dy * dy;
            if (distance < best) {best = distance; nearest = i; selected_fraction = fraction;}
        }
        output[q] = stations[nearest] + .5 * selected_fraction;
    }
}

extern "C" int rmua_response_integrate(
    const double* position, const double* velocity, const double* commands,
    int command_count, const double* scenarios, int scenario_count,
    const double* path, int path_count, const double* height_path, int height_count, double latency, double hold,
    double deceleration, double feedback, double lift_gain, double command_max, double duration, double height_gain, int coupling_limited, double xy_error_max,
    const double* control_periods, const double* applied, int has_applied, int capacity, double* output, double* times) {
    if (command_count < 1 || scenario_count < 1 || capacity < 3 || deceleration <= 0.) return -1;
    const int states = command_count * scenario_count;
    std::vector<double> p(states * 3), v(states * 3), u(states * 3), arc(height_count);
    for (int i = 1; i < height_count; ++i) {
        const double dx = height_path[i * 3] - height_path[(i - 1) * 3];
        const double dy = height_path[i * 3 + 1] - height_path[(i - 1) * 3 + 1];
        arc[i] = arc[i - 1] + std::sqrt(dx * dx + dy * dy);
    }
    for (int state = 0; state < states; ++state) {
        for (int axis = 0; axis < 3; ++axis) {
            const int offset = state * 3 + axis;
            u[offset] = commands[(state / scenario_count) * 3 + axis];
            v[offset] = velocity[axis];
            p[offset] = position[axis];
            output[offset] = position[axis];
        }
    }
    times[0] = 0.;
    const double step = .08;
    int count = 1;
    if (!has_applied) {
        for (int state = 0; state < states; ++state) {
            for (int axis = 0; axis < 3; ++axis) {
                const int offset = state * 3 + axis;
                p[offset] += latency * velocity[axis];
                output[states * 3 + offset] = p[offset];
            }
        }
        times[count++] = latency;
    } else {
        double delayed = 0.;
        while (delayed < latency - 1e-12) {
            if (count >= capacity) return -2;
            const double interval = std::min(step, latency - delayed);
            for (int state = 0; state < states; ++state) {
                const int offset = state * 3, scenario = state % scenario_count;
                const double ex = applied[0] - v[offset], ey = applied[1] - v[offset + 1];
                const double effective[3] = {applied[0], applied[1], applied[2] - scenarios[scenario * 3 + 2] * (ex * ex + ey * ey)};
                for (int axis = 0; axis < 3; ++axis) {
                    const double tau = scenarios[scenario * 3 + (axis == 2 ? 1 : 0)];
                    const double decay = std::exp(-interval / tau);
                    p[offset + axis] += effective[axis] * interval + (v[offset + axis] - effective[axis]) * tau * (1. - decay);
                    v[offset + axis] = effective[axis] + (v[offset + axis] - effective[axis]) * decay;
                    output[(count * states + state) * 3 + axis] = p[offset + axis];
                }
            }
            delayed += interval; times[count++] = delayed;
        }
    }
    double elapsed = 0.;
    std::vector<double> next_control(scenario_count,hold),last_control(scenario_count,0.),control_dt(scenario_count);
    std::vector<bool> update(scenario_count,false);
    while (elapsed < duration) {
        if (count >= capacity) return -2;
        for (int scenario=0;scenario<scenario_count;++scenario) {
            const double period=control_periods[scenario];
            update[scenario]=elapsed>=hold && (period==0. || elapsed+1e-9>=next_control[scenario]);
            if (update[scenario]) {
                control_dt[scenario]=period==0. ? step : std::min(.35,elapsed-last_control[scenario]);
                last_control[scenario]=elapsed;
                next_control[scenario]=elapsed+period;
            }
        }
        double max_u = 0., max_v = 0.;
        for (int state = 0; state < states; ++state) {
            const int offset = state * 3, scenario = state % scenario_count;
            double* ps = p.data() + offset;
            double* vs = v.data() + offset;
            double* us = u.data() + offset;
            if (update[scenario]) {
                double tx, ty;
                braking_target(path, path_count, ps, vs, feedback, tx, ty);
                const double dx = tx - us[0], dy = ty - us[1];
                const double fraction = std::min(1., deceleration * control_dt[scenario] /
                    std::max(1e-9, std::sqrt(dx * dx + dy * dy)));
                us[0] += fraction * dx; us[1] += fraction * dy;
                const double vertical = vertical_target(height_path, arc, ps, vs, height_gain);
                if (coupling_limited && lift_gain > 0.) {
                    const double budget = std::min(xy_error_max,std::sqrt(std::max(0., command_max - vertical) / lift_gain));
                    const double ex = us[0] - vs[0], ey = us[1] - vs[1];
                    const double scale = std::min(1., budget / std::max(1e-9, std::hypot(ex, ey)));
                    us[0] = vs[0] + ex * scale; us[1] = vs[1] + ey * scale;
                }
                const double ex = us[0] - vs[0], ey = us[1] - vs[1];
                us[2] = clip(vertical + lift_gain * (ex * ex + ey * ey), -4., command_max);
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
