#include <gtest/gtest.h>
#include <icarus/core/ConditionParser.hpp>
#include <icarus/signal/Registry.hpp>

using namespace icarus;

// ============================================================================
// Tokenizer Tests
// ============================================================================

TEST(Tokenizer, SimpleComparison) {
    Tokenizer tokenizer("x < 10");
    auto tokens = tokenizer.Tokenize();

    ASSERT_EQ(tokens.size(), 4u); // identifier, <, number, EOF
    EXPECT_EQ(tokens[0].type, TokenType::Identifier);
    EXPECT_EQ(tokens[0].value, "x");
    EXPECT_EQ(tokens[1].type, TokenType::Less);
    EXPECT_EQ(tokens[2].type, TokenType::Number);
    EXPECT_EQ(tokens[2].value, "10");
    EXPECT_EQ(tokens[3].type, TokenType::Eof);
}

TEST(Tokenizer, TwoCharOperators) {
    Tokenizer tokenizer("a <= b >= c == d != e");
    auto tokens = tokenizer.Tokenize();

    ASSERT_EQ(tokens.size(), 10u);
    EXPECT_EQ(tokens[1].type, TokenType::LessEqual);
    EXPECT_EQ(tokens[3].type, TokenType::GreaterEqual);
    EXPECT_EQ(tokens[5].type, TokenType::Equal);
    EXPECT_EQ(tokens[7].type, TokenType::NotEqual);
}

TEST(Tokenizer, BooleanOperators) {
    Tokenizer tokenizer("a AND b OR NOT c");
    auto tokens = tokenizer.Tokenize();

    ASSERT_EQ(tokens.size(), 7u);
    EXPECT_EQ(tokens[1].type, TokenType::And);
    EXPECT_EQ(tokens[3].type, TokenType::Or);
    EXPECT_EQ(tokens[4].type, TokenType::Not);
}

TEST(Tokenizer, SymbolicBooleanOperators) {
    Tokenizer tokenizer("a && b || !c");
    auto tokens = tokenizer.Tokenize();

    ASSERT_EQ(tokens.size(), 7u);
    EXPECT_EQ(tokens[1].type, TokenType::And);
    EXPECT_EQ(tokens[3].type, TokenType::Or);
    EXPECT_EQ(tokens[4].type, TokenType::Not);
}

TEST(Tokenizer, DottedIdentifiers) {
    Tokenizer tokenizer("Vehicle.Nav.altitude > 1000");
    auto tokens = tokenizer.Tokenize();

    ASSERT_EQ(tokens.size(), 4u);
    EXPECT_EQ(tokens[0].type, TokenType::Identifier);
    EXPECT_EQ(tokens[0].value, "Vehicle.Nav.altitude");
}

TEST(Tokenizer, NegativeNumbers) {
    Tokenizer tokenizer("x > -5.5");
    auto tokens = tokenizer.Tokenize();

    ASSERT_EQ(tokens.size(), 4u);
    EXPECT_EQ(tokens[2].type, TokenType::Number);
    EXPECT_EQ(tokens[2].value, "-5.5");
}

TEST(Tokenizer, ScientificNotation) {
    Tokenizer tokenizer("x < 1e-6");
    auto tokens = tokenizer.Tokenize();

    ASSERT_EQ(tokens.size(), 4u);
    EXPECT_EQ(tokens[2].type, TokenType::Number);
    EXPECT_EQ(tokens[2].value, "1e-6");
}

TEST(Tokenizer, Parentheses) {
    Tokenizer tokenizer("(a < 1) AND (b > 2)");
    auto tokens = tokenizer.Tokenize();

    ASSERT_EQ(tokens.size(), 12u);
    EXPECT_EQ(tokens[0].type, TokenType::LeftParen);
    EXPECT_EQ(tokens[4].type, TokenType::RightParen);
}

TEST(Tokenizer, UnexpectedCharacter) {
    Tokenizer tokenizer("x @ y");
    EXPECT_THROW(tokenizer.Tokenize(), ConditionError);
}

// ============================================================================
// Parser Tests
// ============================================================================

TEST(ConditionParser, SimpleComparison) {
    ConditionParser<double> parser;
    auto condition = parser.Parse("x < 10");

    EXPECT_TRUE(condition.IsValid());
    EXPECT_EQ(condition.ToString(), "x < 10.000000");
}

TEST(ConditionParser, AllComparisonOperators) {
    ConditionParser<double> parser;

    EXPECT_NO_THROW(parser.Parse("a < 1"));
    EXPECT_NO_THROW(parser.Parse("a <= 1"));
    EXPECT_NO_THROW(parser.Parse("a > 1"));
    EXPECT_NO_THROW(parser.Parse("a >= 1"));
    EXPECT_NO_THROW(parser.Parse("a == 1"));
    EXPECT_NO_THROW(parser.Parse("a != 1"));
}

TEST(ConditionParser, AndExpression) {
    ConditionParser<double> parser;
    auto condition = parser.Parse("x > 5 AND y < 3");

    EXPECT_TRUE(condition.IsValid());
    EXPECT_EQ(condition.ToString(), "(x > 5.000000 AND y < 3.000000)");
}

TEST(ConditionParser, OrExpression) {
    ConditionParser<double> parser;
    auto condition = parser.Parse("x < 0 OR x > 100");

    EXPECT_TRUE(condition.IsValid());
    EXPECT_EQ(condition.ToString(), "(x < 0.000000 OR x > 100.000000)");
}

TEST(ConditionParser, NotExpression) {
    ConditionParser<double> parser;
    auto condition = parser.Parse("NOT x < 0");

    EXPECT_TRUE(condition.IsValid());
    EXPECT_EQ(condition.ToString(), "NOT x < 0.000000");
}

TEST(ConditionParser, ComplexExpression) {
    ConditionParser<double> parser;
    auto condition = parser.Parse("(a > 1 AND b < 2) OR c == 0");

    EXPECT_TRUE(condition.IsValid());
}

TEST(ConditionParser, SignalToSignalComparison) {
    ConditionParser<double> parser;
    auto condition = parser.Parse("Vehicle.altitude > Ground.elevation");

    EXPECT_TRUE(condition.IsValid());
}

TEST(ConditionParser, MissingOperator) {
    ConditionParser<double> parser;
    EXPECT_THROW(parser.Parse("x y"), ConditionError);
}

TEST(ConditionParser, MissingRightOperand) {
    ConditionParser<double> parser;
    EXPECT_THROW(parser.Parse("x <"), ConditionError);
}

TEST(ConditionParser, UnmatchedParenthesis) {
    ConditionParser<double> parser;
    EXPECT_THROW(parser.Parse("(x < 1"), ConditionError);
}

TEST(ConditionParser, ExtraTokens) {
    ConditionParser<double> parser;
    EXPECT_THROW(parser.Parse("x < 1 2"), ConditionError);
}

// ============================================================================
// Evaluation Tests
// ============================================================================

class ConditionEvaluationTest : public ::testing::Test {
  protected:
    void SetUp() override {
        // Set up registry with test signals
        registry_.set_current_component("Test");
        registry_.register_output("x", &x_, "", "Test signal x");
        registry_.register_output("y", &y_, "", "Test signal y");
        registry_.register_output("Vehicle.altitude", &altitude_, "m", "Altitude");
        registry_.register_output("Propulsion.fuel_mass", &fuel_mass_, "kg", "Fuel mass");
        registry_.clear_current_component();
    }

    SignalRegistry<double> registry_;
    double x_ = 0.0;
    double y_ = 0.0;
    double altitude_ = 0.0;
    double fuel_mass_ = 0.0;
};

TEST_F(ConditionEvaluationTest, LessThan) {
    ConditionParser<double> parser;
    auto condition = parser.Parse("x < 10");

    x_ = 5.0;
    EXPECT_TRUE(condition.Evaluate(registry_));

    x_ = 15.0;
    EXPECT_FALSE(condition.Evaluate(registry_));

    x_ = 10.0;
    EXPECT_FALSE(condition.Evaluate(registry_)); // Not less than
}

TEST_F(ConditionEvaluationTest, LessThanOrEqual) {
    ConditionParser<double> parser;
    auto condition = parser.Parse("x <= 10");

    x_ = 5.0;
    EXPECT_TRUE(condition.Evaluate(registry_));

    x_ = 10.0;
    EXPECT_TRUE(condition.Evaluate(registry_)); // Equal

    x_ = 15.0;
    EXPECT_FALSE(condition.Evaluate(registry_));
}

TEST_F(ConditionEvaluationTest, GreaterThan) {
    ConditionParser<double> parser;
    auto condition = parser.Parse("x > 10");

    x_ = 15.0;
    EXPECT_TRUE(condition.Evaluate(registry_));

    x_ = 5.0;
    EXPECT_FALSE(condition.Evaluate(registry_));
}

TEST_F(ConditionEvaluationTest, Equal) {
    ConditionParser<double> parser;
    auto condition = parser.Parse("x == 10");

    x_ = 10.0;
    EXPECT_TRUE(condition.Evaluate(registry_));

    x_ = 10.001;
    EXPECT_FALSE(condition.Evaluate(registry_));
}

TEST_F(ConditionEvaluationTest, NotEqual) {
    ConditionParser<double> parser;
    auto condition = parser.Parse("x != 10");

    x_ = 5.0;
    EXPECT_TRUE(condition.Evaluate(registry_));

    x_ = 10.0;
    EXPECT_FALSE(condition.Evaluate(registry_));
}

TEST_F(ConditionEvaluationTest, AndLogic) {
    ConditionParser<double> parser;
    auto condition = parser.Parse("x > 5 AND y < 10");

    x_ = 7.0;
    y_ = 5.0;
    EXPECT_TRUE(condition.Evaluate(registry_));

    x_ = 3.0; // x not > 5
    EXPECT_FALSE(condition.Evaluate(registry_));

    x_ = 7.0;
    y_ = 15.0; // y not < 10
    EXPECT_FALSE(condition.Evaluate(registry_));
}

TEST_F(ConditionEvaluationTest, OrLogic) {
    ConditionParser<double> parser;
    auto condition = parser.Parse("x < 0 OR x > 100");

    x_ = -5.0;
    EXPECT_TRUE(condition.Evaluate(registry_));

    x_ = 150.0;
    EXPECT_TRUE(condition.Evaluate(registry_));

    x_ = 50.0;
    EXPECT_FALSE(condition.Evaluate(registry_));
}

TEST_F(ConditionEvaluationTest, NotLogic) {
    ConditionParser<double> parser;
    auto condition = parser.Parse("NOT x < 0");

    x_ = 5.0;
    EXPECT_TRUE(condition.Evaluate(registry_));

    x_ = -5.0;
    EXPECT_FALSE(condition.Evaluate(registry_));
}

TEST_F(ConditionEvaluationTest, DottedSignalNames) {
    ConditionParser<double> parser;
    auto condition = parser.Parse("Vehicle.altitude > 1000");

    altitude_ = 5000.0;
    EXPECT_TRUE(condition.Evaluate(registry_));

    altitude_ = 500.0;
    EXPECT_FALSE(condition.Evaluate(registry_));
}

TEST_F(ConditionEvaluationTest, SignalToSignalComparison) {
    ConditionParser<double> parser;
    auto condition = parser.Parse("x > y");

    x_ = 10.0;
    y_ = 5.0;
    EXPECT_TRUE(condition.Evaluate(registry_));

    x_ = 3.0;
    EXPECT_FALSE(condition.Evaluate(registry_));
}

TEST_F(ConditionEvaluationTest, ComplexCondition) {
    ConditionParser<double> parser;
    auto condition = parser.Parse("(x > 0 AND y > 0) OR (x < 0 AND y < 0)");

    // Both positive
    x_ = 5.0;
    y_ = 5.0;
    EXPECT_TRUE(condition.Evaluate(registry_));

    // Both negative
    x_ = -5.0;
    y_ = -5.0;
    EXPECT_TRUE(condition.Evaluate(registry_));

    // Mixed signs
    x_ = 5.0;
    y_ = -5.0;
    EXPECT_FALSE(condition.Evaluate(registry_));
}

TEST_F(ConditionEvaluationTest, PhaseManagerStyleCondition) {
    ConditionParser<double> parser;
    auto condition = parser.Parse("Propulsion.fuel_mass < 0.01");

    fuel_mass_ = 100.0;
    EXPECT_FALSE(condition.Evaluate(registry_)); // Still have fuel

    fuel_mass_ = 0.005;
    EXPECT_TRUE(condition.Evaluate(registry_)); // Fuel depleted
}

TEST_F(ConditionEvaluationTest, SignalNotFound) {
    ConditionParser<double> parser;
    auto condition = parser.Parse("NonExistent.signal < 10");

    EXPECT_THROW(condition.Evaluate(registry_), SignalNotFoundError);
}

TEST_F(ConditionEvaluationTest, EmptyCondition) {
    CompiledCondition<double> empty;
    EXPECT_FALSE(empty.IsValid());
    EXPECT_THROW(empty.Evaluate(registry_), ConditionError);
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST(ConditionParser, NegativeNumberInCondition) {
    ConditionParser<double> parser;
    auto condition = parser.Parse("x > -100");
    EXPECT_TRUE(condition.IsValid());
}

TEST(ConditionParser, ScientificNotationInCondition) {
    ConditionParser<double> parser;
    auto condition = parser.Parse("x < 1e-6");
    EXPECT_TRUE(condition.IsValid());
}

TEST(ConditionParser, LowercaseKeywords) {
    ConditionParser<double> parser;
    auto condition = parser.Parse("x > 0 and y > 0 or z > 0");
    EXPECT_TRUE(condition.IsValid());
}

TEST(ConditionParser, DeepNesting) {
    ConditionParser<double> parser;
    auto condition = parser.Parse("((a < 1) AND (b < 2)) OR ((c < 3) AND (d < 4))");
    EXPECT_TRUE(condition.IsValid());
}
