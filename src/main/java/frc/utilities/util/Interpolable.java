package frc.utilities.util;

public interface Interpolable<T> {
    T interpolate(T other, double t);
}
