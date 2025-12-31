#pragma once

/**
 * @file AsciiTable.hpp
 * @brief ASCII table generator with box-drawing characters
 *
 * Part of Phase 2.5: ASCII-Rich Logging.
 */

#include <icarus/io/log/Console.hpp>

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

namespace icarus {

/**
 * @brief ASCII table generator with box-drawing characters
 *
 * Example output:
 * ┌────────────────────────┬─────────┬─────────────────────────────────────┐
 * │ SIGNAL NAME            │ UNIT    │ DESCRIPTION                         │
 * ├────────────────────────┼─────────┼─────────────────────────────────────┤
 * │ Gravity.force.x        │ N       │ Gravity force (x)                   │
 * │ Gravity.force.y        │ N       │ Gravity force (y)                   │
 * └────────────────────────┴─────────┴─────────────────────────────────────┘
 */
class AsciiTable {
  public:
    enum class Align { Left, Right, Center };

    struct Column {
        std::string header;
        std::size_t width = 0; ///< 0 = auto-size
        Align align = Align::Left;
    };

    /// Add a column definition
    void AddColumn(const std::string &header, std::size_t width = 0, Align align = Align::Left) {
        Column col;
        col.header = header;
        col.width = width;
        col.align = align;
        columns_.push_back(col);
    }

    /// Add a row of data
    void AddRow(const std::vector<std::string> &cells) { rows_.push_back(cells); }

    /// Generate the formatted table string
    [[nodiscard]] std::string Render() const {
        if (columns_.empty()) {
            return "";
        }

        // Calculate column widths
        std::vector<std::size_t> widths = CalculateWidths();

        std::ostringstream oss;

        // Top border
        oss << RenderTopBorder(widths) << "\n";

        // Header row (always left-aligned)
        oss << RenderHeaderRow(GetHeaders(), widths) << "\n";

        // Header separator
        oss << RenderHeaderSeparator(widths) << "\n";

        // Data rows
        for (const auto &row : rows_) {
            oss << RenderRow(row, widths) << "\n";
        }

        // Bottom border
        oss << RenderBottomBorder(widths) << "\n";

        return oss.str();
    }

    /// Clear all rows (keep columns)
    void ClearRows() { rows_.clear(); }

    /// Clear everything
    void Clear() {
        columns_.clear();
        rows_.clear();
    }

  private:
    std::vector<Column> columns_;
    std::vector<std::vector<std::string>> rows_;

    [[nodiscard]] std::vector<std::string> GetHeaders() const {
        std::vector<std::string> headers;
        headers.reserve(columns_.size());
        for (const auto &col : columns_) {
            headers.push_back(col.header);
        }
        return headers;
    }

    [[nodiscard]] std::vector<std::size_t> CalculateWidths() const {
        std::vector<std::size_t> widths;
        widths.reserve(columns_.size());

        for (std::size_t i = 0; i < columns_.size(); ++i) {
            std::size_t width = columns_[i].width;
            if (width == 0) {
                // Auto-size: max of header and all data
                width = columns_[i].header.size();
                for (const auto &row : rows_) {
                    if (i < row.size()) {
                        width = std::max(width, row[i].size());
                    }
                }
            }
            widths.push_back(width);
        }
        return widths;
    }

    [[nodiscard]] std::string RenderTopBorder(const std::vector<std::size_t> &widths) const {
        std::ostringstream oss;
        oss << BoxChars::TopLeft;
        for (std::size_t i = 0; i < widths.size(); ++i) {
            for (std::size_t j = 0; j < widths[i] + 2; ++j) {
                oss << BoxChars::Horizontal;
            }
            if (i < widths.size() - 1) {
                oss << BoxChars::TeeDown;
            }
        }
        oss << BoxChars::TopRight;
        return oss.str();
    }

    [[nodiscard]] std::string RenderHeaderSeparator(const std::vector<std::size_t> &widths) const {
        std::ostringstream oss;
        oss << BoxChars::TeeRight;
        for (std::size_t i = 0; i < widths.size(); ++i) {
            for (std::size_t j = 0; j < widths[i] + 2; ++j) {
                oss << BoxChars::Horizontal;
            }
            if (i < widths.size() - 1) {
                oss << BoxChars::Cross;
            }
        }
        oss << BoxChars::TeeLeft;
        return oss.str();
    }

    [[nodiscard]] std::string RenderBottomBorder(const std::vector<std::size_t> &widths) const {
        std::ostringstream oss;
        oss << BoxChars::BottomLeft;
        for (std::size_t i = 0; i < widths.size(); ++i) {
            for (std::size_t j = 0; j < widths[i] + 2; ++j) {
                oss << BoxChars::Horizontal;
            }
            if (i < widths.size() - 1) {
                oss << BoxChars::TeeUp;
            }
        }
        oss << BoxChars::BottomRight;
        return oss.str();
    }

    [[nodiscard]] std::string RenderRow(const std::vector<std::string> &cells,
                                        const std::vector<std::size_t> &widths) const {
        std::ostringstream oss;
        oss << BoxChars::Vertical;
        for (std::size_t i = 0; i < widths.size(); ++i) {
            std::string cell = (i < cells.size()) ? cells[i] : "";
            Align align = (i < columns_.size()) ? columns_[i].align : Align::Left;
            oss << " " << AlignCell(cell, widths[i], align) << " ";
            oss << BoxChars::Vertical;
        }
        return oss.str();
    }

    /// Render header row (always left-aligned regardless of column alignment)
    [[nodiscard]] std::string RenderHeaderRow(const std::vector<std::string> &cells,
                                              const std::vector<std::size_t> &widths) const {
        std::ostringstream oss;
        oss << BoxChars::Vertical;
        for (std::size_t i = 0; i < widths.size(); ++i) {
            std::string cell = (i < cells.size()) ? cells[i] : "";
            oss << " " << AlignCell(cell, widths[i], Align::Left) << " ";
            oss << BoxChars::Vertical;
        }
        return oss.str();
    }

    /// Calculate display width of a UTF-8 string (codepoints, not bytes)
    [[nodiscard]] static std::size_t DisplayWidth(const std::string &text) {
        std::size_t width = 0;
        for (std::size_t i = 0; i < text.size(); ++i) {
            unsigned char c = static_cast<unsigned char>(text[i]);
            // Count only lead bytes (not continuation bytes 10xxxxxx)
            if ((c & 0xC0) != 0x80) {
                ++width;
            }
        }
        return width;
    }

    [[nodiscard]] static std::string AlignCell(const std::string &text, std::size_t width,
                                               Align align) {
        std::size_t display_width = DisplayWidth(text);
        if (display_width >= width) {
            return text.substr(0, width);
        }

        std::size_t padding = width - display_width;
        switch (align) {
        case Align::Left:
            return text + std::string(padding, ' ');
        case Align::Right:
            return std::string(padding, ' ') + text;
        case Align::Center: {
            std::size_t left = padding / 2;
            std::size_t right = padding - left;
            return std::string(left, ' ') + text + std::string(right, ' ');
        }
        }
        return text;
    }
};

} // namespace icarus
