#pragma once

/**
 * @file ConditionParser.hpp
 * @brief Signal-based condition expression parser and evaluator
 *
 * Part of Phase 6.1: Phase Manager.
 * Parses conditions like "Propulsion.fuel_mass < 0.01 AND Vehicle.altitude > 1000"
 * and evaluates them against the SignalRegistry.
 */

#include <icarus/core/Error.hpp>
#include <icarus/signal/Registry.hpp>

#include <cctype>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace icarus {

// ============================================================================
// Tokens
// ============================================================================

enum class TokenType {
    // Literals
    Number,     // 0.01, 100, -5.5
    Identifier, // Vehicle.Nav.altitude

    // Comparison operators
    Less,         // <
    LessEqual,    // <=
    Greater,      // >
    GreaterEqual, // >=
    Equal,        // ==
    NotEqual,     // !=

    // Boolean operators
    And, // AND, &&
    Or,  // OR, ||
    Not, // NOT, !

    // Grouping
    LeftParen,  // (
    RightParen, // )

    // End of input
    Eof
};

struct Token {
    TokenType type;
    std::string value;
    std::size_t position = 0;

    Token(TokenType t, std::string v, std::size_t pos = 0)
        : type(t), value(std::move(v)), position(pos) {}
};

// ============================================================================
// Condition Nodes (AST)
// ============================================================================

template <typename Scalar> class ConditionNode {
  public:
    virtual ~ConditionNode() = default;
    [[nodiscard]] virtual bool Evaluate(const SignalRegistry<Scalar> &registry) const = 0;
    [[nodiscard]] virtual std::string ToString() const = 0;
};

/// Comparison: signal <op> value or signal <op> signal
template <typename Scalar> class ComparisonNode : public ConditionNode<Scalar> {
  public:
    enum class Op { Less, LessEqual, Greater, GreaterEqual, Equal, NotEqual };

    ComparisonNode(std::string lhs, Op op, std::variant<double, std::string> rhs)
        : lhs_(std::move(lhs)), op_(op), rhs_(std::move(rhs)) {}

    [[nodiscard]] bool Evaluate(const SignalRegistry<Scalar> &registry) const override {
        double lhs_val = GetValue(registry, lhs_);
        double rhs_val = std::holds_alternative<double>(rhs_)
                             ? std::get<double>(rhs_)
                             : GetValue(registry, std::get<std::string>(rhs_));

        switch (op_) {
        case Op::Less:
            return lhs_val < rhs_val;
        case Op::LessEqual:
            return lhs_val <= rhs_val;
        case Op::Greater:
            return lhs_val > rhs_val;
        case Op::GreaterEqual:
            return lhs_val >= rhs_val;
        case Op::Equal:
            return lhs_val == rhs_val;
        case Op::NotEqual:
            return lhs_val != rhs_val;
        }
        return false;
    }

    [[nodiscard]] std::string ToString() const override {
        std::string op_str;
        switch (op_) {
        case Op::Less:
            op_str = "<";
            break;
        case Op::LessEqual:
            op_str = "<=";
            break;
        case Op::Greater:
            op_str = ">";
            break;
        case Op::GreaterEqual:
            op_str = ">=";
            break;
        case Op::Equal:
            op_str = "==";
            break;
        case Op::NotEqual:
            op_str = "!=";
            break;
        }
        std::string rhs_str = std::holds_alternative<double>(rhs_)
                                  ? std::to_string(std::get<double>(rhs_))
                                  : std::get<std::string>(rhs_);
        return lhs_ + " " + op_str + " " + rhs_str;
    }

  private:
    std::string lhs_;
    Op op_;
    std::variant<double, std::string> rhs_;

    [[nodiscard]] static double GetValue(const SignalRegistry<Scalar> &registry,
                                         const std::string &name) {
        // GetByName returns const Scalar&, convert to double for comparison
        if constexpr (std::is_same_v<Scalar, double>) {
            return registry.GetByName(name);
        } else {
            // For symbolic types, we need numeric evaluation
            // This is a runtime check - symbolic conditions require numeric context
            throw ConditionError("Cannot evaluate symbolic signals in conditions: " + name);
        }
    }
};

/// Boolean AND
template <typename Scalar> class AndNode : public ConditionNode<Scalar> {
  public:
    AndNode(std::unique_ptr<ConditionNode<Scalar>> left,
            std::unique_ptr<ConditionNode<Scalar>> right)
        : left_(std::move(left)), right_(std::move(right)) {}

    [[nodiscard]] bool Evaluate(const SignalRegistry<Scalar> &registry) const override {
        return left_->Evaluate(registry) && right_->Evaluate(registry);
    }

    [[nodiscard]] std::string ToString() const override {
        return "(" + left_->ToString() + " AND " + right_->ToString() + ")";
    }

  private:
    std::unique_ptr<ConditionNode<Scalar>> left_;
    std::unique_ptr<ConditionNode<Scalar>> right_;
};

/// Boolean OR
template <typename Scalar> class OrNode : public ConditionNode<Scalar> {
  public:
    OrNode(std::unique_ptr<ConditionNode<Scalar>> left,
           std::unique_ptr<ConditionNode<Scalar>> right)
        : left_(std::move(left)), right_(std::move(right)) {}

    [[nodiscard]] bool Evaluate(const SignalRegistry<Scalar> &registry) const override {
        return left_->Evaluate(registry) || right_->Evaluate(registry);
    }

    [[nodiscard]] std::string ToString() const override {
        return "(" + left_->ToString() + " OR " + right_->ToString() + ")";
    }

  private:
    std::unique_ptr<ConditionNode<Scalar>> left_;
    std::unique_ptr<ConditionNode<Scalar>> right_;
};

/// Boolean NOT
template <typename Scalar> class NotNode : public ConditionNode<Scalar> {
  public:
    explicit NotNode(std::unique_ptr<ConditionNode<Scalar>> operand)
        : operand_(std::move(operand)) {}

    [[nodiscard]] bool Evaluate(const SignalRegistry<Scalar> &registry) const override {
        return !operand_->Evaluate(registry);
    }

    [[nodiscard]] std::string ToString() const override { return "NOT " + operand_->ToString(); }

  private:
    std::unique_ptr<ConditionNode<Scalar>> operand_;
};

// ============================================================================
// Tokenizer
// ============================================================================

class Tokenizer {
  public:
    explicit Tokenizer(std::string_view input) : input_(input), pos_(0) {}

    [[nodiscard]] std::vector<Token> Tokenize() {
        std::vector<Token> tokens;

        while (!AtEnd()) {
            SkipWhitespace();
            if (AtEnd())
                break;

            std::size_t start = pos_;
            char c = Peek();

            // Two-character operators
            if (c == '<' && PeekNext() == '=') {
                Advance();
                Advance();
                tokens.emplace_back(TokenType::LessEqual, "<=", start);
            } else if (c == '>' && PeekNext() == '=') {
                Advance();
                Advance();
                tokens.emplace_back(TokenType::GreaterEqual, ">=", start);
            } else if (c == '=' && PeekNext() == '=') {
                Advance();
                Advance();
                tokens.emplace_back(TokenType::Equal, "==", start);
            } else if (c == '!' && PeekNext() == '=') {
                Advance();
                Advance();
                tokens.emplace_back(TokenType::NotEqual, "!=", start);
            } else if (c == '&' && PeekNext() == '&') {
                Advance();
                Advance();
                tokens.emplace_back(TokenType::And, "&&", start);
            } else if (c == '|' && PeekNext() == '|') {
                Advance();
                Advance();
                tokens.emplace_back(TokenType::Or, "||", start);
            }
            // Single-character operators
            else if (c == '<') {
                Advance();
                tokens.emplace_back(TokenType::Less, "<", start);
            } else if (c == '>') {
                Advance();
                tokens.emplace_back(TokenType::Greater, ">", start);
            } else if (c == '!') {
                Advance();
                tokens.emplace_back(TokenType::Not, "!", start);
            } else if (c == '(') {
                Advance();
                tokens.emplace_back(TokenType::LeftParen, "(", start);
            } else if (c == ')') {
                Advance();
                tokens.emplace_back(TokenType::RightParen, ")", start);
            }
            // Numbers (including negative)
            else if (std::isdigit(c) || (c == '-' && std::isdigit(PeekNext()))) {
                tokens.push_back(ScanNumber());
            }
            // Identifiers and keywords
            else if (std::isalpha(c) || c == '_') {
                tokens.push_back(ScanIdentifier());
            } else {
                throw ConditionError("Unexpected character '" + std::string(1, c) +
                                     "' at position " + std::to_string(pos_));
            }
        }

        tokens.emplace_back(TokenType::Eof, "", pos_);
        return tokens;
    }

  private:
    std::string_view input_;
    std::size_t pos_;

    [[nodiscard]] bool AtEnd() const { return pos_ >= input_.size(); }

    [[nodiscard]] char Peek() const { return AtEnd() ? '\0' : input_[pos_]; }

    [[nodiscard]] char PeekNext() const {
        return (pos_ + 1 >= input_.size()) ? '\0' : input_[pos_ + 1];
    }

    char Advance() { return input_[pos_++]; }

    void SkipWhitespace() {
        while (!AtEnd() && std::isspace(Peek())) {
            Advance();
        }
    }

    [[nodiscard]] Token ScanNumber() {
        std::size_t start = pos_;
        std::string value;

        // Optional negative sign
        if (Peek() == '-') {
            value += Advance();
        }

        // Integer part
        while (!AtEnd() && std::isdigit(Peek())) {
            value += Advance();
        }

        // Decimal part
        if (Peek() == '.' && std::isdigit(PeekNext())) {
            value += Advance(); // consume '.'
            while (!AtEnd() && std::isdigit(Peek())) {
                value += Advance();
            }
        }

        // Scientific notation
        if (Peek() == 'e' || Peek() == 'E') {
            value += Advance();
            if (Peek() == '+' || Peek() == '-') {
                value += Advance();
            }
            while (!AtEnd() && std::isdigit(Peek())) {
                value += Advance();
            }
        }

        return Token(TokenType::Number, value, start);
    }

    [[nodiscard]] Token ScanIdentifier() {
        std::size_t start = pos_;
        std::string value;

        // First character: letter or underscore
        while (!AtEnd() && (std::isalnum(Peek()) || Peek() == '_' || Peek() == '.')) {
            value += Advance();
        }

        // Check for keywords
        if (value == "AND" || value == "and") {
            return Token(TokenType::And, value, start);
        }
        if (value == "OR" || value == "or") {
            return Token(TokenType::Or, value, start);
        }
        if (value == "NOT" || value == "not") {
            return Token(TokenType::Not, value, start);
        }

        return Token(TokenType::Identifier, value, start);
    }
};

// ============================================================================
// Parser (Recursive Descent)
// ============================================================================

/**
 * @brief Compiled condition ready for evaluation
 *
 * Holds the parsed AST and can evaluate against a SignalRegistry.
 */
template <typename Scalar> class CompiledCondition {
  public:
    CompiledCondition() = default;
    explicit CompiledCondition(std::unique_ptr<ConditionNode<Scalar>> root)
        : root_(std::move(root)) {}

    /// Evaluate the condition against the registry
    [[nodiscard]] bool Evaluate(const SignalRegistry<Scalar> &registry) const {
        if (!root_) {
            throw ConditionError("Cannot evaluate empty condition");
        }
        return root_->Evaluate(registry);
    }

    /// Check if condition is valid (has been parsed)
    [[nodiscard]] bool IsValid() const { return root_ != nullptr; }

    /// Get string representation
    [[nodiscard]] std::string ToString() const { return root_ ? root_->ToString() : "<empty>"; }

  private:
    std::unique_ptr<ConditionNode<Scalar>> root_;
};

/**
 * @brief Parser for condition expressions
 *
 * Grammar:
 *   expression  -> or_expr
 *   or_expr     -> and_expr (OR and_expr)*
 *   and_expr    -> not_expr (AND not_expr)*
 *   not_expr    -> NOT not_expr | primary
 *   primary     -> comparison | '(' expression ')'
 *   comparison  -> identifier ('<'|'<='|'>'|'>='|'=='|'!=') (number | identifier)
 */
template <typename Scalar> class ConditionParser {
  public:
    /**
     * @brief Parse a condition string
     *
     * @param condition The condition expression (e.g., "x < 10 AND y > 5")
     * @return CompiledCondition ready for evaluation
     * @throws ConditionError on parse failure
     */
    [[nodiscard]] CompiledCondition<Scalar> Parse(const std::string &condition) {
        Tokenizer tokenizer(condition);
        tokens_ = tokenizer.Tokenize();
        current_ = 0;

        auto root = ParseOrExpr();

        if (!IsAtEnd()) {
            throw ConditionError("Unexpected token '" + Peek().value + "' at position " +
                                 std::to_string(Peek().position));
        }

        return CompiledCondition<Scalar>(std::move(root));
    }

  private:
    std::vector<Token> tokens_;
    std::size_t current_ = 0;

    [[nodiscard]] const Token &Peek() const { return tokens_[current_]; }

    [[nodiscard]] const Token &Previous() const { return tokens_[current_ - 1]; }

    [[nodiscard]] bool IsAtEnd() const { return Peek().type == TokenType::Eof; }

    Token Advance() {
        if (!IsAtEnd())
            current_++;
        return Previous();
    }

    [[nodiscard]] bool Check(TokenType type) const {
        if (IsAtEnd())
            return false;
        return Peek().type == type;
    }

    bool Match(TokenType type) {
        if (Check(type)) {
            Advance();
            return true;
        }
        return false;
    }

    Token Consume(TokenType type, const std::string &message) {
        if (Check(type))
            return Advance();
        throw ConditionError(message + " at position " + std::to_string(Peek().position));
    }

    // or_expr -> and_expr (OR and_expr)*
    [[nodiscard]] std::unique_ptr<ConditionNode<Scalar>> ParseOrExpr() {
        auto left = ParseAndExpr();

        while (Match(TokenType::Or)) {
            auto right = ParseAndExpr();
            left = std::make_unique<OrNode<Scalar>>(std::move(left), std::move(right));
        }

        return left;
    }

    // and_expr -> not_expr (AND not_expr)*
    [[nodiscard]] std::unique_ptr<ConditionNode<Scalar>> ParseAndExpr() {
        auto left = ParseNotExpr();

        while (Match(TokenType::And)) {
            auto right = ParseNotExpr();
            left = std::make_unique<AndNode<Scalar>>(std::move(left), std::move(right));
        }

        return left;
    }

    // not_expr -> NOT not_expr | primary
    [[nodiscard]] std::unique_ptr<ConditionNode<Scalar>> ParseNotExpr() {
        if (Match(TokenType::Not)) {
            auto operand = ParseNotExpr();
            return std::make_unique<NotNode<Scalar>>(std::move(operand));
        }
        return ParsePrimary();
    }

    // primary -> comparison | '(' expression ')'
    [[nodiscard]] std::unique_ptr<ConditionNode<Scalar>> ParsePrimary() {
        if (Match(TokenType::LeftParen)) {
            auto expr = ParseOrExpr();
            Consume(TokenType::RightParen, "Expected ')' after expression");
            return expr;
        }

        return ParseComparison();
    }

    // comparison -> identifier ('<'|'<='|'>'|'>='|'=='|'!=') (number | identifier)
    [[nodiscard]] std::unique_ptr<ConditionNode<Scalar>> ParseComparison() {
        Token lhs = Consume(TokenType::Identifier, "Expected signal name");

        typename ComparisonNode<Scalar>::Op op;
        if (Match(TokenType::Less)) {
            op = ComparisonNode<Scalar>::Op::Less;
        } else if (Match(TokenType::LessEqual)) {
            op = ComparisonNode<Scalar>::Op::LessEqual;
        } else if (Match(TokenType::Greater)) {
            op = ComparisonNode<Scalar>::Op::Greater;
        } else if (Match(TokenType::GreaterEqual)) {
            op = ComparisonNode<Scalar>::Op::GreaterEqual;
        } else if (Match(TokenType::Equal)) {
            op = ComparisonNode<Scalar>::Op::Equal;
        } else if (Match(TokenType::NotEqual)) {
            op = ComparisonNode<Scalar>::Op::NotEqual;
        } else {
            throw ConditionError("Expected comparison operator at position " +
                                 std::to_string(Peek().position));
        }

        std::variant<double, std::string> rhs;
        if (Match(TokenType::Number)) {
            rhs = std::stod(Previous().value);
        } else if (Match(TokenType::Identifier)) {
            rhs = Previous().value;
        } else {
            throw ConditionError("Expected number or signal name at position " +
                                 std::to_string(Peek().position));
        }

        return std::make_unique<ComparisonNode<Scalar>>(lhs.value, op, std::move(rhs));
    }
};

} // namespace icarus
