#pragma once
#include <cstdint>
#include <chrono>

namespace v2x_interface { namespace metrics {

class link_metrics_t {
public:
    link_metrics_t() { reset(); }

    void reset() {
        // ===== cumulative (frame-level: 기존 의미 유지) =====
        tx_frames_ = rx_frames_ok_ = rx_missing_ = rx_dup_ = rx_reorder_ = 0;
        tx_bytes_acc_ = rx_bytes_acc_ = 0;

        // ===== cumulative (message-level: 1ms 단위) =====
        tx_msgs_ = rx_msgs_ok_ = rx_msg_missing_ = rx_msg_dup_ = rx_msg_reorder_ = 0;
        tx_msg_bytes_acc_ = rx_msg_bytes_acc_ = 0;

        // ===== cumulative (chunk-level: 1200B 전송조각) =====
        tx_chunks_ = rx_chunks_ = 0;
        tx_chunk_bytes_acc_ = rx_chunk_bytes_acc_ = 0;

        // ===== window baselines =====
        last_pub_tp_ = now_us();

        last_tx_bytes_pub_ = last_rx_bytes_pub_ = 0;
        last_tx_frames_pub_ = last_rx_frames_pub_ = 0;
        last_rx_missing_pub_ = last_rx_dup_pub_ = last_rx_reorder_pub_ = 0;

        last_tx_msg_bytes_pub_ = last_rx_msg_bytes_pub_ = 0;
        last_tx_msgs_pub_ = last_rx_msgs_pub_ = 0;
        last_rx_msg_missing_pub_ = last_rx_msg_dup_pub_ = last_rx_msg_reorder_pub_ = 0;

        last_tx_chunk_bytes_pub_ = last_rx_chunk_bytes_pub_ = 0;
        last_tx_chunks_pub_ = last_rx_chunks_pub_ = 0;

        // ===== seq tracking =====
        rx_seq_inited_ = false;
        last_rx_seq_   = 0;

        rx_msg_seq_inited_ = false;
        last_rx_msg_seq_   = 0;

        current_snapshot_ = {};
    }

    // ---------- Frame-level (기존) ----------
    // 호출은 같은 strand(또는 같은 스레드)에서만 하세요.
    void on_tx(uint16_t /*seq*/, std::size_t bytes) {
        ++tx_frames_;
        tx_bytes_acc_ += static_cast<uint64_t>(bytes);
    }

    // seq: 16-bit 순번(프레임 단위)
    void on_rx(uint16_t seq, std::size_t bytes) {
        ++rx_frames_ok_;
        rx_bytes_acc_ += static_cast<uint64_t>(bytes);

        if (!rx_seq_inited_) {
            rx_seq_inited_ = true;
            last_rx_seq_ = seq;
            return;
        }

        const uint16_t prev = last_rx_seq_;
        if (seq == prev) {
            ++rx_dup_;
            return;
        }

        // 16-bit 모듈로 거리 (앞으로 얼마나 갔는지)
        const uint16_t ahead = static_cast<uint16_t>(seq - prev);
        if (ahead > 0) {
            // 정상순서(+ 미수신 계산)
            if (ahead > 1) rx_missing_ += static_cast<uint64_t>(ahead - 1);
            last_rx_seq_ = seq;
        } else {
            // 뒤로 갔음 (wrap이 아닌 역행 = 재정렬로 간주)
            ++rx_reorder_;
            // last_rx_seq_는 유지 (가장 큰 seq가 기준)
        }
    }

    // ---------- Message-level (1ms 전송 메시지) ----------
    // msg_seq: 32-bit 증가값(1ms 주기 메시지 시퀀스)
    void on_tx_msg(uint32_t /*msg_seq*/, std::size_t bytes) {
        ++tx_msgs_;
        tx_msg_bytes_acc_ += static_cast<uint64_t>(bytes);
    }
    void on_rx_msg(uint32_t msg_seq, std::size_t bytes) {
        ++rx_msgs_ok_;
        rx_msg_bytes_acc_ += static_cast<uint64_t>(bytes);

        if (!rx_msg_seq_inited_) {
            rx_msg_seq_inited_ = true;
            last_rx_msg_seq_ = msg_seq;
            return;
        }

        const uint32_t prev = last_rx_msg_seq_;
        if (msg_seq == prev) {
            ++rx_msg_dup_;
            return;
        }

        // 앞/뒤 판정(32-bit wrap 지원):
        // unsigned 덧셈/뺄셈의 성질을 이용해 "앞으로 얼마나 이동했는지" 계산
        const uint32_t ahead = msg_seq - prev; // unsigned wrap OK
        if (ahead > 0 && ahead < 0x80000000u) {
            // 정상 순서. 누락 계산
            if (ahead > 1) rx_msg_missing_ += static_cast<uint64_t>(ahead - 1);
            last_rx_msg_seq_ = msg_seq;
        } else {
            // 과거로 역행(재정렬)
            ++rx_msg_reorder_;
            // 기준은 가장 큰 seq 유지
        }
    }

    // ---------- Chunk-level (1200B 조각) ----------
    void on_tx_chunk(std::size_t bytes) {
        ++tx_chunks_;
        tx_chunk_bytes_acc_ += static_cast<uint64_t>(bytes);
    }
    void on_rx_chunk(std::size_t bytes) {
        ++rx_chunks_;
        rx_chunk_bytes_acc_ += static_cast<uint64_t>(bytes);
    }

    struct snapshot {
        // ===== window metrics (이번 샘플링 구간) =====
        // frame-level
        double   tx_bps = 0.0;   // frame on_tx() 기준 bytes
        double   rx_bps = 0.0;
        uint64_t tx_frames_delta = 0;
        uint64_t rx_frames_delta = 0;
        uint64_t rx_missing_delta = 0;
        uint64_t rx_dup_delta = 0;
        uint64_t rx_reorder_delta = 0;
        double   pdr_rx = 1.0;   // ok / (ok + missing)

        // message-level
        double   tx_msg_bps = 0.0;
        double   rx_msg_bps = 0.0;
        uint64_t tx_msgs_delta = 0;
        uint64_t rx_msgs_delta = 0;
        uint64_t rx_msg_missing_delta = 0;
        uint64_t rx_msg_dup_delta = 0;
        uint64_t rx_msg_reorder_delta = 0;
        double   pdr_rx_msg = 1.0;

        // chunk-level
        double   tx_chunk_bps = 0.0;
        double   rx_chunk_bps = 0.0;
        uint64_t tx_chunks_delta = 0;
        uint64_t rx_chunks_delta = 0;

        // ===== cumulative (총계) =====
        // frame-level
        uint64_t tx_frames = 0;
        uint64_t rx_frames = 0;
        uint64_t rx_missing = 0;
        uint64_t rx_dup = 0;
        uint64_t rx_reorder = 0;
        uint64_t tx_bytes = 0;
        uint64_t rx_bytes = 0;

        // message-level
        uint64_t tx_msgs = 0;
        uint64_t rx_msgs = 0;
        uint64_t rx_msg_missing = 0;
        uint64_t rx_msg_dup = 0;
        uint64_t rx_msg_reorder = 0;
        uint64_t tx_msg_bytes = 0;
        uint64_t rx_msg_bytes = 0;

        // chunk-level
        uint64_t tx_chunks = 0;
        uint64_t rx_chunks = 0;
        uint64_t tx_chunk_bytes = 0;
        uint64_t rx_chunk_bytes = 0;
    };

    // 샘플링 간격(dt) 내 윈도우 지표 계산 + 윈도우 베이스라인 갱신
    snapshot sample_and_reset_window() {
        const auto now = now_us();
        double dt_s = static_cast<double>(now - last_pub_tp_) / 1e6;
        if (dt_s <= 0.0) dt_s = 1e-9; // 0으로 나눔 방지(극단보호)

        // ===== frame-level window =====
        const uint64_t tx_bytes_win = tx_bytes_acc_ - last_tx_bytes_pub_;
        const uint64_t rx_bytes_win = rx_bytes_acc_ - last_rx_bytes_pub_;
        const uint64_t tx_frames_win = tx_frames_ - last_tx_frames_pub_;
        const uint64_t rx_frames_win = rx_frames_ok_ - last_rx_frames_pub_;
        const uint64_t rx_missing_win = rx_missing_ - last_rx_missing_pub_;
        const uint64_t rx_dup_win = rx_dup_ - last_rx_dup_pub_;
        const uint64_t rx_reorder_win = rx_reorder_ - last_rx_reorder_pub_;

        last_tx_bytes_pub_ = tx_bytes_acc_;
        last_rx_bytes_pub_ = rx_bytes_acc_;
        last_tx_frames_pub_ = tx_frames_;
        last_rx_frames_pub_ = rx_frames_ok_;
        last_rx_missing_pub_ = rx_missing_;
        last_rx_dup_pub_ = rx_dup_;
        last_rx_reorder_pub_ = rx_reorder_;

        // ===== message-level window =====
        const uint64_t tx_msg_bytes_win = tx_msg_bytes_acc_ - last_tx_msg_bytes_pub_;
        const uint64_t rx_msg_bytes_win = rx_msg_bytes_acc_ - last_rx_msg_bytes_pub_;
        const uint64_t tx_msgs_win = tx_msgs_ - last_tx_msgs_pub_;
        const uint64_t rx_msgs_win = rx_msgs_ok_ - last_rx_msgs_pub_;
        const uint64_t rx_msg_missing_win = rx_msg_missing_ - last_rx_msg_missing_pub_;
        const uint64_t rx_msg_dup_win = rx_msg_dup_ - last_rx_msg_dup_pub_;
        const uint64_t rx_msg_reorder_win = rx_msg_reorder_ - last_rx_msg_reorder_pub_;

        last_tx_msg_bytes_pub_ = tx_msg_bytes_acc_;
        last_rx_msg_bytes_pub_ = rx_msg_bytes_acc_;
        last_tx_msgs_pub_ = tx_msgs_;
        last_rx_msgs_pub_ = rx_msgs_ok_;
        last_rx_msg_missing_pub_ = rx_msg_missing_;
        last_rx_msg_dup_pub_ = rx_msg_dup_;
        last_rx_msg_reorder_pub_ = rx_msg_reorder_;

        // ===== chunk-level window =====
        const uint64_t tx_chunk_bytes_win = tx_chunk_bytes_acc_ - last_tx_chunk_bytes_pub_;
        const uint64_t rx_chunk_bytes_win = rx_chunk_bytes_acc_ - last_rx_chunk_bytes_pub_;
        const uint64_t tx_chunks_win = tx_chunks_ - last_tx_chunks_pub_;
        const uint64_t rx_chunks_win = rx_chunks_ - last_rx_chunks_pub_;

        last_tx_chunk_bytes_pub_ = tx_chunk_bytes_acc_;
        last_rx_chunk_bytes_pub_ = rx_chunk_bytes_acc_;
        last_tx_chunks_pub_ = tx_chunks_;
        last_rx_chunks_pub_ = rx_chunks_;

        last_pub_tp_ = now;

        snapshot s;

        // ----- frame-level -----
        const uint64_t expected_rx_win = rx_frames_win + rx_missing_win;
        s.pdr_rx = (expected_rx_win > 0) ? static_cast<double>(rx_frames_win) / static_cast<double>(expected_rx_win) : 1.0;
        s.tx_bps = (tx_bytes_win * 8.0) / dt_s;
        s.rx_bps = (rx_bytes_win * 8.0) / dt_s;
        s.tx_frames_delta = tx_frames_win;
        s.rx_frames_delta = rx_frames_win;
        s.rx_missing_delta = rx_missing_win;
        s.rx_dup_delta = rx_dup_win;
        s.rx_reorder_delta = rx_reorder_win;

        s.tx_frames = tx_frames_;
        s.rx_frames = rx_frames_ok_;
        s.rx_missing = rx_missing_;
        s.rx_dup = rx_dup_;
        s.rx_reorder = rx_reorder_;
        s.tx_bytes = tx_bytes_acc_;
        s.rx_bytes = rx_bytes_acc_;

        // ----- message-level -----
        const uint64_t expected_rx_msg_win = rx_msgs_win + rx_msg_missing_win;
        s.pdr_rx_msg = (expected_rx_msg_win > 0) ? static_cast<double>(rx_msgs_win) / static_cast<double>(expected_rx_msg_win) : 1.0;
        s.tx_msg_bps = (tx_msg_bytes_win * 8.0) / dt_s;
        s.rx_msg_bps = (rx_msg_bytes_win * 8.0) / dt_s;
        s.tx_msgs_delta = tx_msgs_win;
        s.rx_msgs_delta = rx_msgs_win;
        s.rx_msg_missing_delta = rx_msg_missing_win;
        s.rx_msg_dup_delta = rx_msg_dup_win;
        s.rx_msg_reorder_delta = rx_msg_reorder_win;

        s.tx_msgs = tx_msgs_;
        s.rx_msgs = rx_msgs_ok_;
        s.rx_msg_missing = rx_msg_missing_;
        s.rx_msg_dup = rx_msg_dup_;
        s.rx_msg_reorder = rx_msg_reorder_;
        s.tx_msg_bytes = tx_msg_bytes_acc_;
        s.rx_msg_bytes = rx_msg_bytes_acc_;

        // ----- chunk-level -----
        s.tx_chunk_bps = (tx_chunk_bytes_win * 8.0) / dt_s;
        s.rx_chunk_bps = (rx_chunk_bytes_win * 8.0) / dt_s;
        s.tx_chunks_delta = tx_chunks_win;
        s.rx_chunks_delta = rx_chunks_win;

        s.tx_chunks = tx_chunks_;
        s.rx_chunks = rx_chunks_;
        s.tx_chunk_bytes = tx_chunk_bytes_acc_;
        s.rx_chunk_bytes = rx_chunk_bytes_acc_;

        current_snapshot_ = s;
        return s;
    }

private:
    static inline uint64_t now_us() {
        using namespace std::chrono;
        return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
    }

    // ===== cumulative: frame-level =====
    uint64_t tx_frames_ = 0, rx_frames_ok_ = 0;
    uint64_t rx_missing_ = 0, rx_dup_ = 0, rx_reorder_ = 0;
    uint64_t tx_bytes_acc_ = 0, rx_bytes_acc_ = 0;

    // ===== cumulative: message-level =====
    uint64_t tx_msgs_ = 0, rx_msgs_ok_ = 0;
    uint64_t rx_msg_missing_ = 0, rx_msg_dup_ = 0, rx_msg_reorder_ = 0;
    uint64_t tx_msg_bytes_acc_ = 0, rx_msg_bytes_acc_ = 0;

    // ===== cumulative: chunk-level =====
    uint64_t tx_chunks_ = 0, rx_chunks_ = 0;
    uint64_t tx_chunk_bytes_acc_ = 0, rx_chunk_bytes_acc_ = 0;

    // ===== window baselines =====
    uint64_t last_pub_tp_ = 0;

    // frame-level
    uint64_t last_tx_bytes_pub_ = 0, last_rx_bytes_pub_ = 0;
    uint64_t last_tx_frames_pub_ = 0, last_rx_frames_pub_ = 0;
    uint64_t last_rx_missing_pub_ = 0, last_rx_dup_pub_ = 0, last_rx_reorder_pub_ = 0;

    // message-level
    uint64_t last_tx_msg_bytes_pub_ = 0, last_rx_msg_bytes_pub_ = 0;
    uint64_t last_tx_msgs_pub_ = 0, last_rx_msgs_pub_ = 0;
    uint64_t last_rx_msg_missing_pub_ = 0, last_rx_msg_dup_pub_ = 0, last_rx_msg_reorder_pub_ = 0;

    // chunk-level
    uint64_t last_tx_chunk_bytes_pub_ = 0, last_rx_chunk_bytes_pub_ = 0;
    uint64_t last_tx_chunks_pub_ = 0, last_rx_chunks_pub_ = 0;

    // ===== sequence state =====
    // frame-level(16-bit)
    bool     rx_seq_inited_ = false;
    uint16_t last_rx_seq_   = 0;

    // message-level(32-bit)
    bool     rx_msg_seq_inited_ = false;
    uint32_t last_rx_msg_seq_   = 0;

    snapshot current_snapshot_{};
};

}} // namespace v2x_interface::metrics
