#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <initializer_list>

// Plano local de referencia para una captura RGB-D fija. Unidades: mm.
// La normal apunta hacia arriba en el sistema optico (Y negativa).
class RgbdReferencePlane
{
public:
    using Vector = std::array<double, 3>;

    void clear() { valid_ = false; }
    bool isValid() const { return valid_; }

    bool fit(const Vector& a, const Vector& b, const Vector& c)
    {
        clear();
        for (const auto& point : {a, b, c})
            for (double value : point) if (!std::isfinite(value)) return false;
        const Vector ab = subtract(b, a), ac = subtract(c, a), bc = subtract(c, b);
        const double lab = length(ab), lac = length(ac);
        // Puntos muy cercanos o casi alineados amplifican el ruido del sensor.
        if (lab < 20 || lac < 20 || length(bc) < 20) return false;
        Vector normal = cross(ab, ac);
        const double magnitude = length(normal);
        if (magnitude / (lab * lac) < 0.08) return false;
        for (double& value : normal) value /= magnitude;
        if (normal[1] > 0) for (double& value : normal) value = -value;
        // Esta prueba esta destinada a una mesa vista desde arriba, no a paredes.
        if (normal[1] > -0.15) return false;
        const double offset = -dot(normal, a);
        if (offset < 20) return false;
        normal_ = normal;
        origin_ = a;
        offset_ = offset;
        Vector lateral = {1 - normal[0] * normal[0],
                          -normal[0] * normal[1], -normal[0] * normal[2]};
        const double size = length(lateral);
        if (size < 1e-6) return false;
        for (double& value : lateral) value /= size;
        lateral_ = lateral;
        forward_ = cross(normal_, lateral_);
        valid_ = true;
        return true;
    }

    double heightMm(const Vector& point) const
    {
        return valid_ ? dot(normal_, point) + offset_
                      : std::numeric_limits<double>::quiet_NaN();
    }

    Vector machinePoint(const Vector& point) const
    {
        if (!valid_) {
            const auto invalid = std::numeric_limits<double>::quiet_NaN();
            return {invalid, invalid, invalid};
        }
        const auto relative = subtract(point, origin_);
        return {dot(lateral_, relative), dot(forward_, relative), heightMm(point)};
    }

private:
    static Vector subtract(const Vector& a, const Vector& b)
    { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
    static double dot(const Vector& a, const Vector& b)
    { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
    static double length(const Vector& a) { return std::sqrt(dot(a, a)); }
    static Vector cross(const Vector& a, const Vector& b)
    { return {a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]}; }
    bool valid_ = false;
    Vector normal_ {}, origin_ {}, lateral_ {}, forward_ {};
    double offset_ = 0;
};
