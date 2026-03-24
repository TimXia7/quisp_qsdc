#ifndef MODULES_QSDCAPPLICATION_H_
#define MODULES_QSDCAPPLICATION_H_

#include "IApplication.h"
#include "modules/Logger/LoggerBase.h"
#include "utils/ComponentProvider.h"

#include "PhotonicQubit_m.h"
#include "backends/interfaces/IQubit.h"

#include <unordered_map>
#include <unordered_set>
#include <string>
#include <vector>

namespace omnetpp {
class cMessage;
class cModule;
}  // namespace omnetpp

namespace quisp::modules {

class StationaryQubit;

class QSDCApplication : public IApplication, public Logger::LoggerBase {
 public:
  QSDCApplication() : provider(utils::ComponentProvider{this}) {}
  ~QSDCApplication() override {}

 protected:
  utils::ComponentProvider provider;
  int my_address = -1;
  bool is_initiator = false;

  // Eve attack settings (attacks phase 1 only)
  bool eve_enabled = false;
  double eve_intercept_probability = 0.0;

  unsigned long active_ruleset_id = 0;
  bool sampling_started = false;
  bool protocol_started = false;
  bool bell_check_started = false;

  // Phase 1: QKD/BB84-style channel test
  int burn_count = 0;
  int burn_current = 0;
  int min_pairs_to_start = 0;
  int sample_target = 0;
  int samples_done = 0;
  int errors = 0;
  int sample_block_size = 8;

  // Phase 2: Bell-pair correlation test
  int bell_sample_target = 0;
  int bell_samples_done = 0;
  int bell_errors = 0;
  int bell_block_size = 8;

  bool waiting_for_sample_block = false;
  bool waiting_for_bell_block = false;

  int current_sample_block_sent = 0;
  int current_bell_block_sent = 0;

  std::vector<int> current_sample_block_indices;
  std::vector<int> current_bell_block_indices;

  bool expect_anti_correlation = false;

  omnetpp::simtime_t start_delay = 0;
  omnetpp::simtime_t poll_interval = 0;
  omnetpp::simtime_t sample_interval = 0;

  struct PendingEntCheck {
    bool awaiting = true;
  };

  struct PendingBellCheck {
    char basis; // 'X' or 'Z'
    int bit;    // measured 0/1
  };

  std::unordered_map<int, PendingEntCheck> pending_checks;       // Phase 1
  std::unordered_map<int, PendingBellCheck> pending_bell_checks;  // Phase 2
  std::unordered_set<int> used_indices;

  void initialize() override;
  void handleMessage(omnetpp::cMessage* msg) override;

  void startOnce();
  void startQSDCProtocol(unsigned long ruleset_id);
  void pollUntilEnoughPairs();

  int countReadyPairsAndCollect(std::vector<int>& out_indices);
  void resetBlockState();

  // Phase 1
  void doNextSample();
  void sendSamplePhoton(int qi, quisp::modules::StationaryQubit* qubit);

  // Phase 2
  void startBellCheckPhase();
  void doNextBellCheck();
  void sendBellCheckRequest(int qi, char basis);
  int measureLocalInBasis(quisp::modules::StationaryQubit* qubit, char basis);

  // Phase 3
  void startDenseTransmission();
  void applyDenseEncoding(quisp::modules::StationaryQubit* qubit, const std::string& bits);
  void sendDensePhoton(int qi, quisp::modules::StationaryQubit* encoded_qubit);
  std::string decodeDensePair(
      quisp::modules::StationaryQubit* local_qubit,
      backends::IQubit* remote_qubit);

  omnetpp::cModule* getLocalEntangledQnic();

  std::vector<int> payload_indices;
  std::vector<std::string> payload_bit_pairs;
  std::vector<std::string> bob_decoded_symbols;
  std::string payload_message_text;
};

}  // namespace quisp::modules

#endif
