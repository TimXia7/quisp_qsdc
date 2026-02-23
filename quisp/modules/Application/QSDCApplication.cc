#include "QSDCApplication.h"

#include <unordered_map>
#include <string>

#include "modules/QNIC/StationaryQubit/StationaryQubit.h"
#include "messages/classical_messages.h"  // Header

using namespace omnetpp;
using namespace quisp::messages;

namespace quisp::modules {

// ---- Simple “protocol” message names ----
static const char *ENT_REQ  = "ENTCHECK_REQ";
static const char *ENT_RESP = "ENTCHECK_RESP";

// Store Alice’s local measurement so we can compare when Bob replies.
struct PendingCheck {
  char basis;       // 'Z' (or 'X' if you extend later)
  int alice_result; // +1 or -1
};
static std::unordered_map<int, PendingCheck> pending_checks;

static inline int eigenToInt(quisp::types::EigenvalueResult r) {
  return (r == quisp::types::EigenvalueResult::PLUS_ONE) ? +1 : -1;
}

// Find a NIC that has statQubit[]: prefer qnic_r[0], else qnic[0]
static cModule *getAnyQnicWithStatQubits(utils::ComponentProvider &provider) {
  auto *qnode = provider.getQNode();
  if (!qnode) return nullptr;
  if (auto *m = qnode->getSubmodule("qnic_r", 0)) return m;
  if (auto *m = qnode->getSubmodule("qnic", 0)) return m;
  return nullptr;
}

void QSDCApplication::initialize() {
  initializeLogger(provider);

  // only keep this module for EndNodes (same logic as Application.cc)
  if (!gate("toRouter")->isConnected() || !gate("fromRouter")->isConnected()) {
    auto *msg = new DeleteThisModule("DeleteThisModule");
    scheduleAt(simTime(), msg);
    return;
  }

  my_address = provider.getNodeAddr();

  auto *qnode = provider.getQNode();
  if (!qnode) {
    EV_ERROR << "[QSDC] No QNode found in initialize()\n";
    return;
  }
  is_initiator = qnode->par("is_initiator").boolValue();

  if (is_initiator) {
    scheduleAt(simTime(), new cMessage("START_ONCE"));
  }
}

void QSDCApplication::handleMessage(cMessage *msg) {
  // If we receive the setup response, QRSA still needs it.
  if (auto *resp = dynamic_cast<ConnectionSetupResponse *>(msg)) {
    EV_INFO << "[QSDC] ConnectionSetupResponse received (ruleset_id="
            << resp->getRuleSet_id() << ")\n";

    startQSDCProtocol(resp->getRuleSet_id());

    delete resp;
    return;
  }

  if (dynamic_cast<DeleteThisModule *>(msg)) {
    delete msg;
    deleteModule();
    return;
  }

  if (strcmp(msg->getName(), "START_ONCE") == 0) {
    delete msg;
    startOnce();
    return;
  }

  // ---- Entanglement-check request (Bob side) ----
  if (strcmp(msg->getName(), ENT_REQ) == 0) {
    // Expect:
    //   src_addr (int), qubit_index (int), basis (string)
    const int src_addr = (int)msg->par("src_addr").longValue();
    const int qi = (int)msg->par("qubit_index").longValue();
    const char basis = msg->par("basis").stringValue()[0];

    auto *qnic = getAnyQnicWithStatQubits(provider);
    if (!qnic) {
      EV_ERROR << "[QSDC] ENT_REQ: no qnic/qnic_r found on this node\n";
      delete msg;
      return;
    }

    auto *sq = qnic->getSubmodule("statQubit", qi);
    if (!sq) {
      EV_ERROR << "[QSDC] ENT_REQ: statQubit[" << qi << "] not found\n";
      delete msg;
      return;
    }

    auto *qubit = check_and_cast<quisp::modules::StationaryQubit *>(sq);

    if (qubit->isLocked()) {
      EV_WARN << "[QSDC] ENT_REQ: qubit[" << qi << "] is locked, skipping\n";
      delete msg;
      return;
    }

    int bob_res = 0;
    if (basis == 'Z') bob_res = eigenToInt(qubit->measureZ());
    else if (basis == 'X') bob_res = eigenToInt(qubit->measureX());
    else {
      EV_ERROR << "[QSDC] ENT_REQ: unsupported basis " << basis << "\n";
      delete msg;
      return;
    }

    EV_INFO << "[QSDC] ENT_REQ: measured qubit_index=" << qi
            << " basis=" << basis << " bob_result=" << bob_res << "\n";

    // Reply to initiator using Header so Router can handle it
    auto *respmsg = new Header(ENT_RESP);
    respmsg->setSrcAddr(my_address);
    respmsg->setDestAddr(src_addr);
    respmsg->setKind(1); // Router delivers kind==1 to the app

    respmsg->addPar("qubit_index") = qi;
    respmsg->addPar("basis") = std::string(1, basis).c_str();
    respmsg->addPar("bob_result") = bob_res;

    send(respmsg, "toRouter");
    delete msg;
    return;
  }

  // ---- Entanglement-check response (Alice side) ----
  if (strcmp(msg->getName(), ENT_RESP) == 0) {
    const int qi = (int)msg->par("qubit_index").longValue();
    const char basis = msg->par("basis").stringValue()[0];
    const int bob_res = (int)msg->par("bob_result").longValue();

    auto it = pending_checks.find(qi);
    if (it == pending_checks.end()) {
      EV_WARN << "[QSDC] ENT_RESP: no pending check for qubit_index=" << qi << "\n";
      delete msg;
      return;
    }

    const int alice_res = it->second.alice_result;

    // If your system sometimes yields anti-correlated Bell states, check both.
    const bool correlated = (alice_res == bob_res);
    const bool anti_correlated = (alice_res == -bob_res);

    EV_INFO << "[QSDC] Entanglement check: qubit_index=" << qi
            << " basis=" << basis
            << " alice=" << alice_res
            << " bob=" << bob_res
            << " => "
            << (correlated ? "CORRELATED" : (anti_correlated ? "ANTI-CORRELATED" : "NO CLEAR RELATION"))
            << "\n";

    pending_checks.erase(it);
    delete msg;
    return;
  }

  // pass-through for other QRSA-related messages
  if (dynamic_cast<ConnectionSetupRequest *>(msg) ||
      dynamic_cast<InternalRuleSetForwarding *>(msg)) {
    logger->logPacket("QSDCApplication::handleMessage", msg);
    send(msg, "toRouter");
    return;
  }

  delete msg;
  error("QSDCApplication: unknown message");
}

void QSDCApplication::startOnce() {
  const int bob_addr = par("bob_addr").intValue();
  const int n_pairs  = par("number_of_bellpair").intValue();

  auto *pk = new ConnectionSetupRequest("ConnSetupRequest");
  pk->setApplicationId(0);

  pk->setActual_srcAddr(my_address);
  pk->setActual_destAddr(bob_addr);

  // routing headers
  pk->setSrcAddr(my_address);
  pk->setDestAddr(my_address);

  pk->setNumber_of_required_Bellpairs(n_pairs);
  pk->setNum_measure(0);
  pk->setKind(7);

  EV_INFO << "[QSDC] Requesting " << n_pairs << " stored Bell pairs from "
          << my_address << " to " << bob_addr << "\n";

  send(pk, "toRouter");
}

void QSDCApplication::startQSDCProtocol(unsigned long ruleset_id) {
  if (!is_initiator) return;

  const int bob_addr = par("bob_addr").intValue();

  auto *qnic = getAnyQnicWithStatQubits(provider);
  if (!qnic) {
    EV_ERROR << "[QSDC] No qnic/qnic_r found on this node\n";
    return;
  }

  const int num_buf = qnic->par("num_buffer").intValue();

  // Start with Z-basis check
  const char basis = 'Z';

  for (int i = 0; i < num_buf; i++) {
    auto *sq = qnic->getSubmodule("statQubit", i);
    if (!sq) continue;

    auto *qubit = check_and_cast<quisp::modules::StationaryQubit *>(sq);

    if (qubit->isLocked()) continue;

    // Alice measures her half
    int alice_res = 0;
    if (basis == 'Z') alice_res = eigenToInt(qubit->measureZ());
    else alice_res = eigenToInt(qubit->measureX());

    pending_checks[i] = PendingCheck{basis, alice_res};

    EV_INFO << "[QSDC] Entanglement check started: qubit_index=" << i
            << " basis=" << basis
            << " alice_result=" << alice_res
            << " (sending Bob request)\n";

    // Send request to Bob using Header so Router can cast it
    auto *req = new Header(ENT_REQ);
    req->setSrcAddr(my_address);
    req->setDestAddr(bob_addr);
    req->setKind(1); // Router delivers kind==1 to the app

    req->addPar("src_addr") = my_address;
    req->addPar("qubit_index") = i;
    req->addPar("basis") = std::string(1, basis).c_str();

    send(req, "toRouter");
    return; // do one check for now
  }

  EV_WARN << "[QSDC] No usable (unlocked) stationary qubits found to test.\n";
}

}  // namespace quisp::modules
