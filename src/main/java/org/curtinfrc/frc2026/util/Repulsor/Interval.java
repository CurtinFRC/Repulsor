/*
 * Copyright (C) 2026 Paul Hodges
 *
 * This file is part of Repulsor.
 *
 * Repulsor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Repulsor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Repulsor. If not, see https://www.gnu.org/licenses/.
 */

package org.curtinfrc.frc2026.util.Repulsor;

import java.util.Comparator;
import java.util.Objects;

/**
 * Provides interval functionality for the Repulsor core Repulsor coordination layer. Use this type
 * from robot code, field profiles, or tests when integrating the corresponding Repulsor subsystem.
 * Coordinates are field-relative unless a method documents robot-relative motion.
 */
public final class Interval<T> {
  /**
   * Defines the bound type values used by the Repulsor core Repulsor coordination layer. Use this
   * type from robot code, field profiles, or tests when integrating the corresponding Repulsor
   * subsystem. Coordinates are field-relative unless a method documents robot-relative motion.
   */
  public enum BoundType {
    OPEN,
    CLOSED
  }

  private final T first;
  private final T second;
  private final Comparator<? super T> cmp;
  private final BoundType lowerBound;
  private final BoundType upperBound;

  private Interval(
      T first, T second, Comparator<? super T> cmp, BoundType lowerBound, BoundType upperBound) {
    Objects.requireNonNull(first, "first");
    Objects.requireNonNull(second, "second");
    Objects.requireNonNull(cmp, "cmp");
    Objects.requireNonNull(lowerBound, "lowerBound");
    Objects.requireNonNull(upperBound, "upperBound");
    if (cmp.compare(first, second) > 0) {
      T swapped = first;
      first = second;
      second = swapped;
    }
    this.first = first;
    this.second = second;
    this.cmp = cmp;
    this.lowerBound = lowerBound;
    this.upperBound = upperBound;
  }

  /**
   * Returns the closed value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @return value produced by this operation.
   */
  public static <T extends Comparable<? super T>> Interval<T> closed(T a, T b) {
    return new Interval<>(a, b, Comparator.naturalOrder(), BoundType.CLOSED, BoundType.CLOSED);
  }

  /**
   * Returns the open value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @return value produced by this operation.
   */
  public static <T extends Comparable<? super T>> Interval<T> open(T a, T b) {
    return new Interval<>(a, b, Comparator.naturalOrder(), BoundType.OPEN, BoundType.OPEN);
  }

  /**
   * Returns the closed open value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @return value produced by this operation.
   */
  public static <T extends Comparable<? super T>> Interval<T> closedOpen(T a, T b) {
    return new Interval<>(a, b, Comparator.naturalOrder(), BoundType.CLOSED, BoundType.OPEN);
  }

  /**
   * Returns the open closed value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @return value produced by this operation.
   */
  public static <T extends Comparable<? super T>> Interval<T> openClosed(T a, T b) {
    return new Interval<>(a, b, Comparator.naturalOrder(), BoundType.OPEN, BoundType.CLOSED);
  }

  /**
   * Returns the of value maintained by this Repulsor component.
   *
   * @param a value used by this operation.
   * @param b value used by this operation.
   * @param comparator value used by this operation.
   * @param lowerBound value used by this operation.
   * @param upperBound value used by this operation.
   * @return value produced by this operation.
   */
  public static <T> Interval<T> of(
      T a, T b, Comparator<? super T> comparator, BoundType lowerBound, BoundType upperBound) {
    return new Interval<>(a, b, comparator, lowerBound, upperBound);
  }

  /**
   * Returns the first value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public T first() {
    return first;
  }

  /**
   * Returns the second value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public T second() {
    return second;
  }

  /**
   * Returns the min value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public T min() {
    return cmp.compare(first, second) <= 0 ? first : second;
  }

  /**
   * Returns the max value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public T max() {
    return cmp.compare(first, second) <= 0 ? second : first;
  }

  /**
   * Returns the is empty value maintained by this Repulsor component.
   *
   * @return value produced by this operation.
   */
  public boolean isEmpty() {
    int c = cmp.compare(first, second);
    if (c == 0) return lowerBound == BoundType.OPEN || upperBound == BoundType.OPEN;
    return false;
  }

  /**
   * Returns the within value maintained by this Repulsor component.
   *
   * @param x distance or field-coordinate value in meters.
   * @return value produced by this operation.
   */
  public boolean within(T x) {
    Objects.requireNonNull(x, "x");

    T lo = min();
    T hi = max();

    int cl = cmp.compare(x, lo);
    int ch = cmp.compare(x, hi);

    boolean okLower = (lowerBound == BoundType.CLOSED) ? (cl >= 0) : (cl > 0);
    boolean okUpper = (upperBound == BoundType.CLOSED) ? (ch <= 0) : (ch < 0);

    return okLower && okUpper;
  }

  /**
   * Returns the contains value maintained by this Repulsor component.
   *
   * @param x distance or field-coordinate value in meters.
   * @return value produced by this operation.
   */
  public boolean contains(T x) {
    return within(x);
  }

  /**
   * Returns the overlaps value maintained by this Repulsor component.
   *
   * @param other value used by this operation.
   * @return value produced by this operation.
   */
  public boolean overlaps(Interval<T> other) {
    Objects.requireNonNull(other, "other");

    if (this.isEmpty() || other.isEmpty()) return false;

    T aLo = this.min(), aHi = this.max();
    T bLo = other.min(), bHi = other.max();

    int left = cmp.compare(aHi, bLo);
    int right = cmp.compare(bHi, aLo);

    if (left < 0 || right < 0) return false;

    if (left == 0)
      return this.upperBound == BoundType.CLOSED && other.lowerBound == BoundType.CLOSED;
    if (right == 0)
      return other.upperBound == BoundType.CLOSED && this.lowerBound == BoundType.CLOSED;

    return true;
  }
}
