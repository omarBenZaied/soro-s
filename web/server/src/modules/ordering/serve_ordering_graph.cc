#include "soro/server/modules/ordering/ordering_module.h"

#include "net/web_server/responses.h"

#include "soro/simulation/ordering_graph.h"

#include "soro/server/cereal/json_archive.h"

namespace soro::server {

std::string serialize_ordering_graph(simulation::ordering_graph const& og) {
  using namespace rapidjson;

  StringBuffer node_buffer;
  Writer<StringBuffer> node_writer(node_buffer);
  node_writer.StartArray();

  StringBuffer edge_buffer;
  Writer<StringBuffer> edge_writer(edge_buffer);
  edge_writer.StartArray();

  for (auto const& node : og.nodes_) {
    node_writer.StartObject();
    node_writer.String("key");
    node_writer.Int(node.id_);
    node_writer.EndObject();

    for (auto const to : node.out_) {
      edge_writer.StartObject();
      edge_writer.String("Source");
      edge_writer.Int(node.id_);
      edge_writer.String("Target");
      edge_writer.Int(to);
      edge_writer.EndObject();
    }
  }

  node_writer.EndArray();
  edge_writer.EndArray();

  std::stringstream result;
  result << R"({ "attributes": {}, "nodes":)" << node_buffer.GetString()
         << R"(, "edges":)" << edge_buffer.GetString() << "}";
  return result.str();
}

net::web_server::string_res_t ordering_module::serve_ordering_graph(
    net::query_router::route_request const& req,
    infrastructure_module const& infra_m,
    timetable_module const& timetable_m) const {
  utls::expect(req.path_params_.size() == 4);

  std::string_view const infra_name = req.path_params_.front();
  auto const infra = infra_m.get_infra(infra_name);
  if (!infra.has_value()) {
    return net::not_found_response(req);
  }

  std::string_view const timetable_name = req.path_params_[1];
  auto const timetable = timetable_m.get_timetable(infra_name, timetable_name);
  if (!timetable.has_value()) {
    return net::not_found_response(req);
  }

  auto const from = str_to_absolute_time(req.path_params_[2]);
  if (!from) return net::bad_request_response(req);

  auto const to = str_to_absolute_time(req.path_params_[3]);
  if (!to) return net::bad_request_response(req);

  tt::interval iv{.start_ = *from, .end_ = *to};

  if (!iv.valid()) return net::bad_request_response(req);

  simulation::ordering_graph const ordering_graph(**infra, **timetable, iv);

  return json_response(req, serialize_ordering_graph(ordering_graph));
}

}  // namespace soro::server