/**
 * @file test_logging.cpp
 * @brief Unit tests for Phase 2.5 ASCII-Rich Logging
 */

#include <icarus/io/AsciiTable.hpp>
#include <icarus/io/Banner.hpp>
#include <icarus/io/Console.hpp>
#include <icarus/io/ErrorHandler.hpp>
#include <icarus/io/LogConfig.hpp>
#include <icarus/io/LogEntry.hpp>
#include <icarus/io/LogService.hpp>
#include <icarus/io/LogSink.hpp>
#include <icarus/io/MissionDebrief.hpp>

#include <gtest/gtest.h>
#include <sstream>
#include <thread>

using namespace icarus;

// =============================================================================
// Console Tests
// =============================================================================

TEST(Console, DefaultColorDetection) {
    Console console;
    // IsTerminal depends on whether test is run in terminal or CI
    // Just verify it returns a bool
    bool is_tty = console.IsTerminal();
    EXPECT_EQ(console.IsColorEnabled(), is_tty);
}

TEST(Console, SetColorEnabled) {
    Console console;
    console.SetColorEnabled(true);
    EXPECT_TRUE(console.IsColorEnabled());

    console.SetColorEnabled(false);
    EXPECT_FALSE(console.IsColorEnabled());
}

TEST(Console, ColorizeStripsWhenDisabled) {
    Console console;
    console.SetColorEnabled(false);

    auto result = console.Colorize("test", AnsiColor::Red);
    EXPECT_EQ(result, "test"); // No color codes
}

TEST(Console, ColorizeAddsWhenEnabled) {
    Console console;
    console.SetColorEnabled(true);

    auto result = console.Colorize("test", AnsiColor::Red);
    EXPECT_TRUE(result.find("\033[31m") != std::string::npos);
    EXPECT_TRUE(result.find("\033[0m") != std::string::npos);
}

TEST(Console, PadRight) {
    EXPECT_EQ(Console::PadRight("foo", 6), "foo   ");
    EXPECT_EQ(Console::PadRight("foobar", 6), "foobar");
    EXPECT_EQ(Console::PadRight("foobar", 4), "foobar"); // No truncation
}

TEST(Console, PadLeft) {
    EXPECT_EQ(Console::PadLeft("foo", 6), "   foo");
    EXPECT_EQ(Console::PadLeft("foobar", 6), "foobar");
}

TEST(Console, PadCenter) {
    EXPECT_EQ(Console::PadCenter("foo", 7), "  foo  ");
    EXPECT_EQ(Console::PadCenter("foo", 6), " foo  ");
}

TEST(Console, HorizontalRule) {
    Console console;
    auto rule = console.HorizontalRule(10, '-');
    EXPECT_EQ(rule, "----------");
}

TEST(Console, LogLevelFiltering) {
    Console console;
    console.SetLogLevel(LogLevel::Warning);
    EXPECT_EQ(console.GetLogLevel(), LogLevel::Warning);

    // Log below threshold - should not output (testing the setting)
    console.SetLogLevel(LogLevel::Trace);
    EXPECT_EQ(console.GetLogLevel(), LogLevel::Trace);
}

// =============================================================================
// LogEntry Tests
// =============================================================================

TEST(LogEntry, CreateWithContext) {
    LogContext ctx;
    ctx.entity = "vehicle1";
    ctx.component = "engine";
    ctx.type = "LiquidEngine";

    auto entry = LogEntry::Create(LogLevel::Warning, 1.5, "Pressure high", ctx);

    EXPECT_EQ(entry.level, LogLevel::Warning);
    EXPECT_EQ(entry.sim_time, 1.5);
    EXPECT_EQ(entry.message, "Pressure high");
    EXPECT_EQ(entry.context.entity, "vehicle1");
    EXPECT_EQ(entry.context.component, "engine");
}

TEST(LogEntry, FormatBasic) {
    LogContext ctx;
    ctx.entity = "v1";
    ctx.component = "eng";

    auto entry = LogEntry::Create(LogLevel::Info, 0.0, "Started", ctx);
    std::string formatted = entry.Format();

    EXPECT_TRUE(formatted.find("[0.000]") != std::string::npos);
    EXPECT_TRUE(formatted.find("[INF]") != std::string::npos);
    EXPECT_TRUE(formatted.find("[v1.eng]") != std::string::npos);
    EXPECT_TRUE(formatted.find("Started") != std::string::npos);
}

TEST(LogEntry, FormatWithoutContext) {
    LogContext ctx; // Empty context

    auto entry = LogEntry::Create(LogLevel::Info, 0.0, "Message", ctx);
    std::string formatted = entry.Format(false);

    EXPECT_TRUE(formatted.find("[0.000]") != std::string::npos);
    EXPECT_TRUE(formatted.find("Message") != std::string::npos);
}

TEST(LogContext, FullPath) {
    LogContext ctx;
    ctx.entity = "ship";
    ctx.component = "engine";

    EXPECT_EQ(ctx.FullPath(), "ship.engine");

    ctx.entity = "";
    EXPECT_EQ(ctx.FullPath(), "engine");
}

TEST(LogContext, IsSet) {
    LogContext ctx;
    EXPECT_FALSE(ctx.IsSet());

    ctx.component = "test";
    EXPECT_TRUE(ctx.IsSet());
}

// =============================================================================
// LogContextManager Tests
// =============================================================================

TEST(LogContextManager, GetSetContext) {
    LogContext ctx;
    ctx.entity = "test_entity";
    ctx.component = "test_component";

    LogContextManager::SetContext(ctx);
    auto retrieved = LogContextManager::GetContext();

    EXPECT_EQ(retrieved.entity, "test_entity");
    EXPECT_EQ(retrieved.component, "test_component");

    LogContextManager::ClearContext();
    auto cleared = LogContextManager::GetContext();
    EXPECT_FALSE(cleared.IsSet());
}

TEST(LogContextManager, ScopedContext) {
    LogContextManager::ClearContext();
    EXPECT_FALSE(LogContextManager::GetContext().IsSet());

    {
        LogContextManager::ScopedContext scope("entity", "comp", "type");
        auto ctx = LogContextManager::GetContext();
        EXPECT_EQ(ctx.entity, "entity");
        EXPECT_EQ(ctx.component, "comp");
        EXPECT_EQ(ctx.type, "type");
    }

    // Context should be cleared after scope
    EXPECT_FALSE(LogContextManager::GetContext().IsSet());
}

TEST(LogContextManager, NestedScopedContext) {
    {
        LogContextManager::ScopedContext outer("outer_entity", "outer_comp");
        EXPECT_EQ(LogContextManager::GetContext().entity, "outer_entity");

        {
            LogContextManager::ScopedContext inner("inner_entity", "inner_comp");
            EXPECT_EQ(LogContextManager::GetContext().entity, "inner_entity");
        }

        // Outer scope restored
        EXPECT_EQ(LogContextManager::GetContext().entity, "outer_entity");
    }

    EXPECT_FALSE(LogContextManager::GetContext().IsSet());
}

// =============================================================================
// LogService Tests
// =============================================================================

TEST(LogService, DefaultImmediateMode) {
    LogService service;
    EXPECT_TRUE(service.IsImmediateMode());
}

TEST(LogService, BufferedMode) {
    LogService service;
    service.SetImmediateMode(false);
    EXPECT_FALSE(service.IsImmediateMode());
}

TEST(LogService, LogLevelFiltering) {
    LogService service;
    service.SetMinLevel(LogLevel::Warning);
    service.SetImmediateMode(false);

    // This should be filtered out
    service.Info(0.0, "This is info");
    EXPECT_EQ(service.PendingCount(), 0);

    // This should pass
    service.Warning(0.0, "This is warning");
    EXPECT_EQ(service.PendingCount(), 1);

    service.Clear();
}

TEST(LogService, ErrorTracking) {
    LogService service;
    service.SetImmediateMode(false);
    service.ClearSinks(); // Remove default sinks to avoid output

    EXPECT_FALSE(service.HasErrors());
    EXPECT_EQ(service.ErrorCount(), 0);

    service.Error(0.0, "Test error");
    EXPECT_TRUE(service.HasErrors());
    EXPECT_EQ(service.ErrorCount(), 1);

    service.Fatal(0.0, "Test fatal");
    EXPECT_TRUE(service.HasFatalErrors());
    EXPECT_EQ(service.FatalCount(), 1);

    service.ResetErrorCounts();
    EXPECT_FALSE(service.HasErrors());
    EXPECT_EQ(service.ErrorCount(), 0);

    service.Clear();
}

TEST(LogService, BufferedScope) {
    LogService service;
    EXPECT_TRUE(service.IsImmediateMode());

    {
        LogService::BufferedScope scope(service);
        EXPECT_FALSE(service.IsImmediateMode());
    }

    // Mode restored
    EXPECT_TRUE(service.IsImmediateMode());
}

TEST(LogService, SinkCallback) {
    LogService service;
    service.SetImmediateMode(false);
    service.SetMinLevel(LogLevel::Info);

    std::vector<LogEntry> received;
    service.AddSink([&received](const std::vector<LogEntry> &entries) {
        for (const auto &e : entries) {
            received.push_back(e);
        }
    });

    service.Info(0.0, "Test message");
    service.Flush();

    ASSERT_EQ(received.size(), 1);
    EXPECT_EQ(received[0].message, "Test message");

    service.Clear();
}

// =============================================================================
// AsciiTable Tests
// =============================================================================

TEST(AsciiTable, RendersCorrectly) {
    AsciiTable table;
    table.AddColumn("NAME", 10);
    table.AddColumn("VALUE", 8);

    table.AddRow({"foo", "123"});
    table.AddRow({"bar", "456"});

    std::string result = table.Render();

    EXPECT_TRUE(result.find("┌") != std::string::npos);
    EXPECT_TRUE(result.find("foo") != std::string::npos);
    EXPECT_TRUE(result.find("456") != std::string::npos);
    EXPECT_TRUE(result.find("└") != std::string::npos);
}

TEST(AsciiTable, AutoSizesColumns) {
    AsciiTable table;
    table.AddColumn("HEADER"); // Auto-size

    table.AddRow({"short"});
    table.AddRow({"this is a longer value"});

    std::string result = table.Render();

    // Should fit the longer value
    EXPECT_TRUE(result.find("this is a longer value") != std::string::npos);
}

TEST(AsciiTable, Alignment) {
    AsciiTable table;
    table.AddColumn("LEFT", 10, AsciiTable::Align::Left);
    table.AddColumn("RIGHT", 10, AsciiTable::Align::Right);
    table.AddColumn("CENTER", 10, AsciiTable::Align::Center);

    table.AddRow({"L", "R", "C"});

    std::string result = table.Render();

    // Just verify it renders without error
    EXPECT_FALSE(result.empty());
    EXPECT_TRUE(result.find("LEFT") != std::string::npos);
}

TEST(AsciiTable, ClearRows) {
    AsciiTable table;
    table.AddColumn("COL");
    table.AddRow({"row1"});
    table.AddRow({"row2"});

    table.ClearRows();
    std::string result = table.Render();

    // Should still have headers but no data rows
    EXPECT_TRUE(result.find("COL") != std::string::npos);
}

// =============================================================================
// Banner Tests
// =============================================================================

TEST(Banner, SplashScreen) {
    std::string splash = Banner::GetSplashScreen("DEBUG", "CASADI");
    EXPECT_TRUE(splash.find("6DOF") != std::string::npos);
    EXPECT_TRUE(splash.find("FLIGHT DYNAMICS") != std::string::npos);
    EXPECT_TRUE(splash.find("DEBUG") != std::string::npos);
    EXPECT_TRUE(splash.find("CASADI") != std::string::npos);
}

TEST(Banner, PhaseHeader) {
    std::string header = Banner::GetPhaseHeader("PROVISION");
    EXPECT_TRUE(header.find("PROVISION") != std::string::npos);
    EXPECT_TRUE(header.find("───") != std::string::npos);
}

TEST(Banner, DebriefHeader) {
    std::string header = Banner::GetDebriefHeader();
    EXPECT_TRUE(header.find("MISSION DEBRIEF") != std::string::npos);
}

// =============================================================================
// MissionDebrief Tests
// =============================================================================

TEST(MissionDebrief, Generate) {
    Console console;
    console.SetColorEnabled(false);

    MissionDebrief debrief(console);
    debrief.SetExitStatus(ExitStatus::Success);
    debrief.SetExitReason("End condition met");
    debrief.SetTiming(10.0, 0.5);

    std::string output = debrief.Generate();

    EXPECT_TRUE(output.find("SUCCESS") != std::string::npos);
    EXPECT_TRUE(output.find("End condition met") != std::string::npos);
    EXPECT_TRUE(output.find("10.00") != std::string::npos);
    EXPECT_TRUE(output.find("Real-Time Factor") != std::string::npos);
}

TEST(MissionDebrief, ProfilingTable) {
    Console console;
    console.SetColorEnabled(false);

    MissionDebrief debrief(console);
    debrief.SetExitStatus(ExitStatus::Success);
    debrief.SetTiming(10.0, 1.0);

    std::vector<ComponentStats> stats;
    ComponentStats s1;
    s1.name = "Component1";
    s1.avg_time_us = 100.0;
    s1.percent_load = 60.0;
    stats.push_back(s1);

    ComponentStats s2;
    s2.name = "Component2";
    s2.avg_time_us = 50.0;
    s2.percent_load = 40.0;
    stats.push_back(s2);

    debrief.SetProfilingData(stats);

    std::string output = debrief.Generate();

    EXPECT_TRUE(output.find("PROFILE HOTSPOTS") != std::string::npos);
    EXPECT_TRUE(output.find("Component1") != std::string::npos);
    EXPECT_TRUE(output.find("Component2") != std::string::npos);
}

// =============================================================================
// LogConfig Tests
// =============================================================================

TEST(LogConfig, Default) {
    auto config = LogConfig::Default();
    EXPECT_EQ(config.console_level, LogLevel::Info);
    EXPECT_TRUE(config.progress_enabled);
    EXPECT_FALSE(config.profiling_enabled);
}

TEST(LogConfig, Quiet) {
    auto config = LogConfig::Quiet();
    EXPECT_EQ(config.console_level, LogLevel::Error);
    EXPECT_FALSE(config.progress_enabled);
    EXPECT_TRUE(config.quiet_mode);
}

TEST(LogConfig, Verbose) {
    auto config = LogConfig::Verbose();
    EXPECT_EQ(config.console_level, LogLevel::Trace);
    EXPECT_TRUE(config.file_enabled);
}

TEST(LogConfig, WithProfiling) {
    auto config = LogConfig::WithProfiling();
    EXPECT_TRUE(config.profiling_enabled);
}

// =============================================================================
// ErrorHandler Tests
// =============================================================================

TEST(ErrorHandler, DefaultPolicies) {
    LogService service;
    service.SetImmediateMode(false);
    service.ClearSinks();

    ErrorHandler handler(service);

    SimulationError warning;
    warning.severity = Severity::WARNING;
    warning.message = "Test warning";
    warning.component = "test";
    warning.time = 0.0;

    auto policy = handler.Report(warning);
    EXPECT_EQ(policy, ErrorPolicy::Continue);

    service.Clear();
}

TEST(ErrorHandler, ErrorCounting) {
    LogService service;
    service.SetImmediateMode(false);
    service.ClearSinks();

    ErrorHandler handler(service);

    SimulationError error;
    error.severity = Severity::ERROR;
    error.message = "Test error";
    error.component = "test";
    error.time = 0.0;

    handler.Report(error);
    handler.Report(error);

    EXPECT_EQ(handler.GetErrorCount(Severity::ERROR), 2);
    EXPECT_EQ(handler.GetTotalErrorCount(), 2);

    handler.Reset();
    EXPECT_EQ(handler.GetErrorCount(Severity::ERROR), 0);

    service.Clear();
}

TEST(ErrorHandler, MaxErrors) {
    LogService service;
    service.SetImmediateMode(false);
    service.ClearSinks();

    ErrorHandler handler(service);
    handler.SetMaxErrors(3);

    SimulationError error;
    error.severity = Severity::ERROR;
    error.message = "Test";
    error.component = "test";

    handler.Report(error);
    handler.Report(error);
    EXPECT_FALSE(handler.ShouldAbort());

    handler.Report(error); // Third error
    EXPECT_TRUE(handler.ShouldAbort());

    service.Clear();
}
