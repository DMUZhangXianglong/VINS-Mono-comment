/*
 * @Author: DMU zhangxianglong
 * @Date: 2024-07-10 09:04:56
 * @LastEditTime: 2024-07-10 11:00:40
 * @LastEditors: DMU zhangxianglong
 * @FilePath: /VINS-Mono-comment/pose_graph/src/utility/tic_toc.h
 * @Description: 
 */
#pragma once // 防止头文件被多次包含的预处理指令

#include <ctime> // 提供了一些与时间
#include <cstdlib> // 随机数生成相关的函数
#include <chrono> // 提供了用于时间测量和处理的类和函数
 
class TicToc
{
  public:
    TicToc()
    {
        tic();
    }

    // 记录当前时间
    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    // 计算时间差值
    double toc()
    {
       
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        // *1000 转为毫秒
        return elapsed_seconds.count() * 1000;
    }

  private:
    // 声明成员变量 用于保存 开始时间 和 结束时间
    std::chrono::time_point<std::chrono::system_clock> start, end;
};
