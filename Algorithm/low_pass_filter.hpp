#ifndef LOW_PASS_FILTER_H_
#define LOW_PASS_FILTER_H_
#include <cmath>

class LowPassFilter
{
public:
    /**
     * @brief 初始化低通滤波器
     * @param cutoff_freq 截止频率(Hz)
     * @param dt          采样周期(s)
     */
    void Init(float cutoff_freq, float dt)
    {
        if (cutoff_freq <= 0.0f)
        {
            alpha_ = 1.0f; // 不滤波
        }
        else
        {
            float rc = 1.0f / (2.0f * M_PI * cutoff_freq);
            alpha_ = dt / (dt + rc);
        }
        output_ = 0.0f;
        initialized_ = true;
    }

    /**
     * @brief 输入一个新值并更新滤波结果
     * @param input 当前输入值
     * @return 滤波后的输出
     */
    float Update(float input)
    {
        if (!initialized_)
            return input; // 未初始化则直接透传
        output_ = alpha_ * input + (1.0f - alpha_) * output_;
        return output_;
    }

    /**
     * @brief 读取当前滤波输出
     */
    float GetOutput() const { return output_; }

    /**
     * @brief 重置滤波器状态
     * @param value 初始化输出值
     */
    void Reset(float value = 0.0f)
    {
        output_ = value;
    }

private:
    float alpha_;       // 滤波系数
    float output_;      // 当前输出
    bool initialized_;  // 是否已初始化
};

#endif // LOW_PASS_FILTER_H_
