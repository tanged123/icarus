#pragma once

/**
 * @file RouteLoader.hpp
 * @brief Loads signal routes from YAML files
 *
 * Part of Phase 4.0: Configuration Infrastructure.
 */

#include <icarus/signal/SignalRouter.hpp>

#include <vulcan/io/YamlNode.hpp>

#include <string>

namespace icarus {
namespace io {

/**
 * @brief Loads signal routes from YAML files
 *
 * Expected YAML structure:
 * @code
 * routes:
 *   - input: Entity.Component.signal
 *     output: Entity.Component.signal
 *     gain: 1.0  # optional, defaults to 1.0
 * @endcode
 */
class RouteLoader {
  public:
    /**
     * @brief Load routes from file
     *
     * @param router SignalRouter to populate
     * @param yaml_path Path to YAML file
     */
    template <typename Scalar>
    static void LoadRoutes(signal::SignalRouter<Scalar> &router, const std::string &yaml_path) {
        auto root = vulcan::io::YamlNode::LoadFile(yaml_path);
        ParseRoutesFromNode(router, root);
    }

    /**
     * @brief Parse routes from YAML string
     *
     * @param router SignalRouter to populate
     * @param yaml_content YAML content as string
     */
    template <typename Scalar>
    static void ParseRoutes(signal::SignalRouter<Scalar> &router, const std::string &yaml_content) {
        auto root = vulcan::io::YamlNode::Parse(yaml_content);
        ParseRoutesFromNode(router, root);
    }

  private:
    template <typename Scalar>
    static void ParseRoutesFromNode(signal::SignalRouter<Scalar> &router,
                                    const vulcan::io::YamlNode &root) {
        if (!root.Has("routes")) {
            return;
        }

        auto routes = root["routes"];
        for (std::size_t i = 0; i < routes.Size(); ++i) {
            auto route_node = routes[i];

            std::string input = route_node.Require<std::string>("input");
            std::string output = route_node.Require<std::string>("output");
            double gain = route_node.Get<double>("gain", 1.0);

            router.AddRoute(input, output, gain);
        }
    }
};

} // namespace io
} // namespace icarus
