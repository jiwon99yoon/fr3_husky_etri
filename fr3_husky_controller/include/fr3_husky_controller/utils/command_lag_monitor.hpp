#pragma once

// Joint position commands written to the robot over the last kMaxLag cycles, to
// compare the robot's executed command (libfranka q_d) with the command written
// `lag` cycles earlier.
//
// Which earlier command q_d shows depends on the franka_hardware pipeline:
//   lag 1: FrankaHardwareInterface (single arm): read -> controller -> write in one
//          cycle, so the state read in cycle k carries c_{k-1}.
//   lag 2: FrankaMultiHardwareInterface (dual arm): the worker reads S_k and then
//          writes c_{k-1} (the controller's previous output) in the same cycle,
//          so S_k carries c_{k-2} and one command is always in flight.
// Comparing with the wrong one makes every moving cycle look like "robot not
// executing the command" (11-17 analysis: the tracker re-syncs every cycle and
// diverges on the dual-arm pipeline).
//
// Statistics: on cycles where the last two commands differ (the robot is commanded
// to move, so the comparison is unambiguous) count whether q_d equals c_{k-1},
// c_{k-2} or neither.  A lag that does not match the hardware shows up there.

#include <array>
#include <cstdint>

#include <Eigen/Core>

namespace fr3_husky_controller::utils
{

class CommandLagMonitor
{
public:
    static constexpr int kMaxLag = 3;

    enum class Match : int { kNone = 0, kLag1 = 1, kLag2 = 2 };

    // Robot at rest at q (q_d = held command = q): every past command is q.
    // Allocates (call outside the 1 kHz loop or once at goal start).
    void reset(const Eigen::VectorXd& q)
    {
        for (auto& c : written_)
        {
            c = q;
        }
        head_ = 0;
        moving_cycles_ = 0;
        lag1_cycles_ = 0;
        lag2_cycles_ = 0;
        other_cycles_ = 0;
        last_match_ = Match::kNone;
    }

    // Command written `lag` cycles ago (1 = the most recent), 1 <= lag <= kMaxLag.
    const Eigen::VectorXd& written(int lag) const
    {
        return written_[static_cast<std::size_t>((head_ - lag + 2 * kMaxLag) % kMaxLag)];
    }

    // Call once per cycle with the q_d read this cycle, before the new command is
    // computed.  Updates the statistics; returns true when q_d differs from the
    // command written `lag` cycles ago by more than tol (the robot did not execute
    // it, or the hardware rate limiter changed it).
    bool check(const Eigen::VectorXd& q_d, int lag, double tol)
    {
        const Eigen::VectorXd& c1 = written(1);
        const Eigen::VectorXd& c2 = written(2);
        last_match_ = Match::kNone;
        if ((c1 - c2).cwiseAbs().maxCoeff() > tol)
        {
            ++moving_cycles_;
            if ((q_d - c1).cwiseAbs().maxCoeff() <= tol)
            {
                ++lag1_cycles_;
                last_match_ = Match::kLag1;
            }
            else if ((q_d - c2).cwiseAbs().maxCoeff() <= tol)
            {
                ++lag2_cycles_;
                last_match_ = Match::kLag2;
            }
            else
            {
                ++other_cycles_;
            }
        }
        return (q_d - written(lag)).cwiseAbs().maxCoeff() > tol;
    }

    // Command written this cycle (same size as reset()'s q: no allocation).
    void push(const Eigen::VectorXd& q)
    {
        written_[static_cast<std::size_t>(head_)] = q;
        head_ = (head_ + 1) % kMaxLag;
    }

    uint64_t movingCycles() const { return moving_cycles_; }
    uint64_t lag1Cycles() const { return lag1_cycles_; }
    uint64_t lag2Cycles() const { return lag2_cycles_; }
    uint64_t otherCycles() const { return other_cycles_; }
    // Result of the last check() (kNone also when the robot was not commanded to move).
    Match lastMatch() const { return last_match_; }

    // Lag that q_d matched in at least `fraction` of at least `min_cycles` moving
    // cycles; 0 when there is not enough evidence.
    int dominantLag(uint64_t min_cycles, double fraction) const
    {
        if (moving_cycles_ < min_cycles)
        {
            return 0;
        }
        const double n = static_cast<double>(moving_cycles_);
        if (static_cast<double>(lag1_cycles_) >= fraction * n)
        {
            return 1;
        }
        if (static_cast<double>(lag2_cycles_) >= fraction * n)
        {
            return 2;
        }
        return 0;
    }

private:
    std::array<Eigen::VectorXd, kMaxLag> written_;
    int head_{0};
    uint64_t moving_cycles_{0};
    uint64_t lag1_cycles_{0};
    uint64_t lag2_cycles_{0};
    uint64_t other_cycles_{0};
    Match last_match_{Match::kNone};
};

}  // namespace fr3_husky_controller::utils
