/**
 * @file config.h
 * @brief 配置读取功能实现
 *
 * 利用nlohmann json库实现
 *
 * @author lqf
 * @date 2024.3.3
 * @version 0.0
 */

#pragma once

#include <fstream>
#include <iostream>
#include <json.hpp>

using njson = nlohmann::json;

/**
 * @brief json配置文件读取类
 */
class Config
{
  private:
    njson data; ///< 从json文件中读取出来的数据

  public:
    /**
     * @brief 配置数据加载函数
     *
     * 将配置数据从给定json文件中加载出来
     *
     * @param path json文件路径
     * @return 配置文件读取成功与否
     */
    bool load(const std::string &path)
    {
        std::ifstream file(path);
        if (!file.is_open())
        {
            std::cerr << "[Config] Failed to open file: " << path << std::endl;
            return false;
        }

        try
        {
            file >> data;
        }
        catch (const njson::parse_error &e)
        {
            std::cerr << "[Config] Parser error: " << e.what() << std::endl;
            return false;
        }

        std::cout << "[Config] Ready!!!" << std::endl;
        return true;
    }

    /**
     * @brief 配置信息读取函数
     *
     * 根据给定key从加载好的data中读取需要的配置信息
     *
     * @tparam T 模板参数类型，指定读取出来的配置信息的数据类型
     * @param key 需要读取的配置信息对应的key值
     * @return 数据类型为T的配置信息
     */
    template <typename T> T get(const std::string &key) const
    {
        try
        {
            if (data.contains(key))
            {
                return data[key].get<T>();
            }
            else
            {
                throw std::runtime_error("Key not found: " + key);
            }
        }
        catch (const njson::exception &e)
        {
            std::cerr << "Error reading key " << key << ": " << e.what() << std::endl;
            throw;
        }
    }
};
