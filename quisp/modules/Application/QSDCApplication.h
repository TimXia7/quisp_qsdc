#ifndef MODULES_QSDCAPPLICATION_H_
#define MODULES_QSDCAPPLICATION_H_

#include "IApplication.h"
#include "modules/Logger/LoggerBase.h"
#include "utils/ComponentProvider.h"

#include <unordered_map>
#include <unordered_set>
#include <set>
#include <map>
#include <string>
#include <vector>

namespace omnetpp {
class cMessage;
class cModule;
}  // namespace omnetpp

namespace quisp::modules {

class QSDCApplication : public IApplication, public Logger::LoggerBase {
 public:
  QSDCApplication() : provider(utils::ComponentProvider{this}) {}
  ~QSDCApplication() override {}

 protected:
  utils::ComponentProvider provider;
  int my_address = -1;
  bool is_initiator = false;

  unsigned long active_ruleset_id = 0;
  bool sampling_started = false;

  int min_pairs_to_start = 0;
  int sample_target = 0;
  int samples_done = 0;
  int errors = 0;

  bool expect_anti_correlation = false;

  omnetpp::simtime_t start_delay = 0;
  omnetpp::simtime_t poll_interval = 0;
  omnetpp::simtime_t sample_interval = 0;

  struct PendingCheck {
    char basis;
    int alice_result;
  };

  std::unordered_map<int, PendingCheck> pending_checks;
  std::unordered_set<int> used_indices;  // indices already consumed by sampling

  // OMNeT lifecycle
  void initialize() override;
  void handleMessage(omnetpp::cMessage* msg) override;

  // Existing flow
  void startOnce();
  void startQSDCProtocol(unsigned long ruleset_id);

  // helpers
  void pollUntilEnoughPairs();
  void doNextSample();

  // Count "ready" pair slots and return their indices.
  int countReadyPairsAndCollect(std::vector<int>& out_indices);

  // Returns the local QNIC module that holds the stationary qubits we should sample.
  // (For your observed two-node setup: Alice uses qnic_r[0], Bob uses qnic[0].)
  omnetpp::cModule* getLocalEntangledQnic();

  void applyDenseEncoding(quisp::modules::StationaryQubit* qubit, const std::string& bits);
  std::string decodeDensePair(
    quisp::modules::StationaryQubit* local_qubit,
    quisp::modules::StationaryQubit* remote_qubit);
  void startDenseTransmission();

  std::vector<int> payload_indices;
  std::vector<std::string> payload_bit_pairs;
  std::vector<std::string> bob_decoded_symbols;
  std::string payload_message_text;
};

Define_Module(QSDCApplication);

}  // namespace quisp::modules

#endif
