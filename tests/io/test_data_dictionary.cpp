#include <filesystem>
#include <gtest/gtest.h>
#include <icarus/core/Error.hpp>
#include <icarus/io/data/DataDictionary.hpp>

TEST(DataDictionary, ToYAML_WriteError) {
    icarus::DataDictionary dd;
    // Try to write to a non-existent directory -> Open error
    std::string invalid_path = "/nonexistent_directory/file.yaml";
    EXPECT_THROW(
        {
            try {
                dd.ToYAML(invalid_path);
            } catch (const icarus::IOError &e) {
                std::string msg = e.what();
                EXPECT_TRUE(msg.find(invalid_path) != std::string::npos);
                EXPECT_TRUE(msg.find("IO:") != std::string::npos);
                throw;
            }
        },
        icarus::IOError);
}

TEST(DataDictionary, ToJSON_WriteError) {
    icarus::DataDictionary dd;
    // Try to write to a non-existent directory -> Open error
    std::string invalid_path = "/nonexistent_directory/file.json";
    EXPECT_THROW(
        {
            try {
                dd.ToJSON(invalid_path);
            } catch (const icarus::IOError &e) {
                std::string msg = e.what();
                EXPECT_TRUE(msg.find(invalid_path) != std::string::npos);
                EXPECT_TRUE(msg.find("IO:") != std::string::npos);
                throw;
            }
        },
        icarus::IOError);
}

TEST(DataDictionary, ToYAML_Success) {
    icarus::DataDictionary dd;
    std::string path = "test_output.yaml";
    EXPECT_NO_THROW(dd.ToYAML(path));
    EXPECT_TRUE(std::filesystem::exists(path));
    std::filesystem::remove(path);
}

TEST(DataDictionary, ToJSON_Success) {
    icarus::DataDictionary dd;
    std::string path = "test_output.json";
    EXPECT_NO_THROW(dd.ToJSON(path));
    EXPECT_TRUE(std::filesystem::exists(path));
    std::filesystem::remove(path);
}
