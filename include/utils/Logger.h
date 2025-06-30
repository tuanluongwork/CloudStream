#pragma once

#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <mutex>
#include <memory>
#include <chrono>
#include <iomanip>

namespace CloudStream {

class Logger {
public:
    enum class Level {
        Debug = 0,
        Info = 1,
        Warning = 2,
        Error = 3,
        Fatal = 4
    };
    
    static Logger& instance() {
        static Logger logger;
        return logger;
    }
    
    void setLevel(Level level) { level_ = level; }
    Level getLevel() const { return level_; }
    
    void setOutputFile(const std::string& filename) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (file_stream_.is_open()) {
            file_stream_.close();
        }
        file_stream_.open(filename, std::ios::app);
    }
    
    template<typename... Args>
    void debug(const std::string& format, Args... args) {
        log(Level::Debug, format, args...);
    }
    
    template<typename... Args>
    void info(const std::string& format, Args... args) {
        log(Level::Info, format, args...);
    }
    
    template<typename... Args>
    void warning(const std::string& format, Args... args) {
        log(Level::Warning, format, args...);
    }
    
    template<typename... Args>
    void error(const std::string& format, Args... args) {
        log(Level::Error, format, args...);
    }
    
    template<typename... Args>
    void fatal(const std::string& format, Args... args) {
        log(Level::Fatal, format, args...);
    }
    
private:
    Logger() = default;
    ~Logger() {
        if (file_stream_.is_open()) {
            file_stream_.close();
        }
    }
    
    template<typename... Args>
    void log(Level level, const std::string& format, Args... args) {
        if (level < level_) return;
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        
        std::stringstream ss;
        ss << "[" << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S") << "] ";
        ss << "[" << levelToString(level) << "] ";
        
        // Simple format string implementation
        std::string formatted = format;
        formatString(formatted, args...);
        ss << formatted;
        
        std::string message = ss.str();
        
        // Output to console
        if (level >= Level::Error) {
            std::cerr << message << std::endl;
        } else {
            std::cout << message << std::endl;
        }
        
        // Output to file if configured
        if (file_stream_.is_open()) {
            file_stream_ << message << std::endl;
            file_stream_.flush();
        }
    }
    
    const char* levelToString(Level level) {
        switch (level) {
            case Level::Debug: return "DEBUG";
            case Level::Info: return "INFO";
            case Level::Warning: return "WARN";
            case Level::Error: return "ERROR";
            case Level::Fatal: return "FATAL";
            default: return "UNKNOWN";
        }
    }
    
    template<typename T, typename... Args>
    void formatString(std::string& format, T&& value, Args&&... args) {
        size_t pos = format.find("{}");
        if (pos != std::string::npos) {
            std::stringstream ss;
            ss << value;
            format.replace(pos, 2, ss.str());
            formatString(format, args...);
        }
    }
    
    void formatString(std::string&) {}
    
    Level level_{Level::Info};
    std::mutex mutex_;
    std::ofstream file_stream_;
};

} // namespace CloudStream 