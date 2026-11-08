#pragma once

#include <array>
#include <string_view>

namespace mesheal {

struct Mesh;

/// Supported physical length units for interpreting or scaling STL vertex coordinates.
enum class Unit {
    Millimeter,
    Centimeter,
    Meter,
    Inch,
    Foot
};

/// Ordered list of all supported units for UI dropdowns and iteration.
inline constexpr std::array<Unit, 5> kAllUnits = {
    Unit::Millimeter,
    Unit::Centimeter,
    Unit::Meter,
    Unit::Inch,
    Unit::Foot
};

/// Short symbol for a length unit (e.g., "mm", "cm", "m", "in", "ft").
[[nodiscard]] constexpr std::string_view unitSymbol(Unit unit) noexcept {
    switch (unit) {
        case Unit::Millimeter: return "mm";
        case Unit::Centimeter: return "cm";
        case Unit::Meter:      return "m";
        case Unit::Inch:       return "in";
        case Unit::Foot:       return "ft";
    }
    return "mm";
}

/// Human-readable display label for a length unit (e.g., "Millimeters (mm)").
[[nodiscard]] constexpr std::string_view unitDisplayName(Unit unit) noexcept {
    switch (unit) {
        case Unit::Millimeter: return "Millimeters (mm)";
        case Unit::Centimeter: return "Centimeters (cm)";
        case Unit::Meter:      return "Meters (m)";
        case Unit::Inch:       return "Inches (in)";
        case Unit::Foot:       return "Feet (ft)";
    }
    return "Millimeters (mm)";
}

/// Number of centimeters per 1.0 of the given unit.
[[nodiscard]] constexpr double unitToCentimeters(Unit unit) noexcept {
    switch (unit) {
        case Unit::Millimeter: return 0.1;
        case Unit::Centimeter: return 1.0;
        case Unit::Meter:      return 100.0;
        case Unit::Inch:       return 2.54;
        case Unit::Foot:       return 30.48;
    }
    return 0.1;
}

/// Converts a length expressed in centimeters into the target unit.
[[nodiscard]] constexpr double centimetersToUnit(double lengthCm, Unit targetUnit) noexcept {
    switch (targetUnit) {
        case Unit::Millimeter: return lengthCm * 10.0;
        case Unit::Centimeter: return lengthCm;
        case Unit::Meter:      return lengthCm * 0.01;
        case Unit::Inch:       return lengthCm / 2.54;
        case Unit::Foot:       return lengthCm / 30.48;
    }
    return lengthCm * 10.0;
}

/// Multiplicative scale factor to convert a length from `fromUnit` to `toUnit`.
[[nodiscard]] constexpr double unitConversionFactor(Unit fromUnit, Unit toUnit) noexcept {
    if (fromUnit == toUnit) {
        return 1.0;
    }
    return centimetersToUnit(unitToCentimeters(fromUnit), toUnit);
}

/// Re-interprets the mesh's active file unit without altering vertex coordinates `V`.
void setFileUnit(Mesh& mesh, Unit newUnit) noexcept;

/// Explicitly scales `mesh.V` from `mesh.fileUnit` into `targetUnit` and updates `mesh.fileUnit`.
void scaleMeshToUnit(Mesh& mesh, Unit targetUnit);

} // namespace mesheal
