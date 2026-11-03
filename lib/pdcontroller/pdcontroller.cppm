module;
#include <concepts>
#include <iostream>
export module adcs.PDController;


export namespace adcs {

template <typename T>
concept HasZeroFunc = requires {
    { T::Zero() } -> std::convertible_to<T>;
};

template <typename T>
concept ZeroInitializable = std::is_arithmetic_v<T> || HasZeroFunc<T>;

template <typename T>
concept Controllable = ZeroInitializable<T> && requires(T t, float f) {
    { t + t } -> std::convertible_to<T>;
    { t - t } -> std::convertible_to<T>;
    { f * t } -> std::convertible_to<T>;
    { t / f } -> std::convertible_to<T>;
};

template <ZeroInitializable T>
constexpr T Initializer() noexcept {
    if constexpr (HasZeroFunc<T>) return T::Zero();
    else return T {};
};

template <Controllable T>
class PDController {

protected:

    float mKP = 0.0f;
    float mKD = 0.0f;

    T mLastError = Initializer<T>();

public:

    const auto  kP        () const { return mKP; }
    const auto  kD        () const { return mKD; }
    const auto& lastError () const { return mLastError; }

    void setKP (const auto aKP) { mKP = aKP; }
    void setKD (const auto aKD) { mKD = aKD; }

    T computeEffort(
        const T& current,
        const T& goal,
        const float deltaTime
    ) {
        if (deltaTime == 0.0f) return current * 0.0f;

        const auto& error = goal - current;
        const auto& deltaError = error - mLastError;
        mLastError = error;

        return mKP * error + mKD * (deltaError / deltaTime);
    }

    T computeEffort(
        const T& error,
        const float deltaTime
    ) {
        if (deltaTime == 0.0f) return error * 0.0f;

        const auto& deltaError = error - mLastError;
        mLastError = error;

        return mKP * error + mKD * (deltaError / deltaTime);
    }

    friend std::ostream& operator<< (std::ostream& ostream, const PDController& instance) {
        return ostream
            << "kP: " << instance.kP() << "\n"
            << "kD: " << instance.kD() << "\n"
            << "lastError: " << instance.lastError() << "\n";
    }
};

} // export namespace adcs
