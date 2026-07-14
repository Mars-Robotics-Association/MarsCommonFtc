package com.ruckig;

/**
 * Port of {@code block.hpp} (upstream v0.17.3).
 * Value semantics for Profile/Interval use explicit {@link Profile#copyFrom}.
 */
public final class Block {

    public static final class Interval {
        public double left, right; // [s]
        public final Profile profile = new Profile(); // Profile corresponding to right (end) time

        public Interval() {}

        public void set(double left, double right) {
            this.left = left;
            this.right = right;
        }

        public void set(Profile profile_left, Profile profile_right) {
            final double left_duration = profile_left.t_sum[6] + profile_left.brake.duration + profile_left.accel.duration;
            final double right_duration = profile_right.t_sum[6] + profile_right.brake.duration + profile_right.accel.duration;
            if (left_duration < right_duration) {
                left = left_duration;
                right = right_duration;
                profile.copyFrom(profile_right);
            } else {
                left = right_duration;
                right = left_duration;
                profile.copyFrom(profile_left);
            }
        }

        public void copyFrom(Interval o) {
            left = o.left;
            right = o.right;
            profile.copyFrom(o.profile);
        }
    }

    public final Profile p_min = new Profile(); // Save min profile so that it doesn't need to be recalculated in Step2
    public double t_min; // [s]

    // Max. 2 intervals can be blocked
    public boolean has_a;
    public boolean has_b;
    public final Interval a = new Interval();
    public final Interval b = new Interval();

    public void clearIntervals() {
        has_a = false;
        has_b = false;
    }

    public void set_min_profile(Profile profile) {
        p_min.copyFrom(profile);
        t_min = p_min.t_sum[6] + p_min.brake.duration + p_min.accel.duration;
        has_a = false;
        has_b = false;
    }

    private static void remove_profile(Profile[] valid_profiles, int[] valid_profile_counter, int index) {
        int n = valid_profile_counter[0];
        for (int i = index; i < n - 1; ++i) {
            valid_profiles[i].copyFrom(valid_profiles[i + 1]);
        }
        valid_profile_counter[0] = n - 1;
    }

    /**
     * @param valid_profile_counter number of valid profiles in the array (mutated when removing)
     */
    public static boolean calculate_block(Block block, Profile[] valid_profiles, int valid_profile_counter) {
        return calculate_block(block, valid_profiles, valid_profile_counter, true);
    }

    public static boolean calculate_block(Block block, Profile[] valid_profiles, int valid_profile_counter, boolean numerical_robust) {
        int[] counterBox = new int[] { valid_profile_counter };

        if (counterBox[0] == 1) {
            block.set_min_profile(valid_profiles[0]);
            return true;

        } else if (counterBox[0] == 2) {
            if (Math.abs(valid_profiles[0].t_sum[6] - valid_profiles[1].t_sum[6]) < 8 * Utils.DBL_EPSILON) {
                block.set_min_profile(valid_profiles[0]);
                return true;
            }

            if (numerical_robust) {
                final int idx_min = (valid_profiles[0].t_sum[6] < valid_profiles[1].t_sum[6]) ? 0 : 1;
                final int idx_else_1 = (idx_min + 1) % 2;

                block.set_min_profile(valid_profiles[idx_min]);
                block.a.set(valid_profiles[idx_min], valid_profiles[idx_else_1]);
                block.has_a = true;
                return true;
            }

        // Only happens due to numerical issues
        } else if (counterBox[0] == 4) {
            // Find "identical" profiles
            if (Math.abs(valid_profiles[0].t_sum[6] - valid_profiles[1].t_sum[6]) < 32 * Utils.DBL_EPSILON
                    && valid_profiles[0].direction != valid_profiles[1].direction) {
                remove_profile(valid_profiles, counterBox, 1);
            } else if (Math.abs(valid_profiles[2].t_sum[6] - valid_profiles[3].t_sum[6]) < 256 * Utils.DBL_EPSILON
                    && valid_profiles[2].direction != valid_profiles[3].direction) {
                remove_profile(valid_profiles, counterBox, 3);
            } else if (Math.abs(valid_profiles[0].t_sum[6] - valid_profiles[3].t_sum[6]) < 256 * Utils.DBL_EPSILON
                    && valid_profiles[0].direction != valid_profiles[3].direction) {
                remove_profile(valid_profiles, counterBox, 3);
            } else {
                return false;
            }

        } else if (counterBox[0] % 2 == 0) {
            return false;
        }

        // Find index of fastest profile
        int idx_min = 0;
        for (int i = 1; i < counterBox[0]; ++i) {
            if (valid_profiles[i].t_sum[6] < valid_profiles[idx_min].t_sum[6]) {
                idx_min = i;
            }
        }

        block.set_min_profile(valid_profiles[idx_min]);

        if (counterBox[0] == 3) {
            final int idx_else_1 = (idx_min + 1) % 3;
            final int idx_else_2 = (idx_min + 2) % 3;

            block.a.set(valid_profiles[idx_else_1], valid_profiles[idx_else_2]);
            block.has_a = true;
            return true;

        } else if (counterBox[0] == 5) {
            final int idx_else_1 = (idx_min + 1) % 5;
            final int idx_else_2 = (idx_min + 2) % 5;
            final int idx_else_3 = (idx_min + 3) % 5;
            final int idx_else_4 = (idx_min + 4) % 5;

            if (valid_profiles[idx_else_1].direction == valid_profiles[idx_else_2].direction) {
                block.a.set(valid_profiles[idx_else_1], valid_profiles[idx_else_2]);
                block.b.set(valid_profiles[idx_else_3], valid_profiles[idx_else_4]);
            } else {
                block.a.set(valid_profiles[idx_else_1], valid_profiles[idx_else_4]);
                block.b.set(valid_profiles[idx_else_2], valid_profiles[idx_else_3]);
            }
            block.has_a = true;
            block.has_b = true;
            return true;
        }

        return false;
    }

    public boolean is_blocked(double t) {
        return (t < t_min)
            || (has_a && a.left < t && t < a.right)
            || (has_b && b.left < t && t < b.right);
    }

    public Profile get_profile(double t) {
        if (has_b && t >= b.right) {
            return b.profile;
        }
        if (has_a && t >= a.right) {
            return a.profile;
        }
        return p_min;
    }

    @Override
    public String toString() {
        String result = "[" + t_min + " ";
        if (has_a) {
            result += a.left + "] [" + a.right + " ";
        }
        if (has_b) {
            result += b.left + "] [" + b.right + " ";
        }
        return result + "-";
    }
}
