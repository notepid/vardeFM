import CVardeFM
import XCTest

/// Minimal host that queues TX audio for the peer to hear.
final class LoopbackHost {
    var lines: [String] = []
    var data = Data()
    var queue: [Float] = []
    /// Samples that really went out. Flushed ones are not counted.
    var played: UInt64 = 0
    var ptt = false
    var modem: OpaquePointer?

    func callbacks() -> VfmCallbacks {
        VfmCallbacks(
            tx_audio: { ctx, samples, n in
                let host = Unmanaged<LoopbackHost>.fromOpaque(ctx!).takeUnretainedValue()
                let count = Int(n)
                let buf = UnsafeBufferPointer(start: samples, count: count)
                host.queue.append(contentsOf: buf)
            },
            tx_flush: { ctx in
                let host = Unmanaged<LoopbackHost>.fromOpaque(ctx!).takeUnretainedValue()
                host.queue.removeAll(keepingCapacity: true)
            },
            ptt: { ctx, on in
                let host = Unmanaged<LoopbackHost>.fromOpaque(ctx!).takeUnretainedValue()
                host.ptt = on
            },
            host_line: { ctx, line in
                let host = Unmanaged<LoopbackHost>.fromOpaque(ctx!).takeUnretainedValue()
                if let line, let s = String(validatingUTF8: line) {
                    host.lines.append(s)
                }
            },
            host_data: { ctx, data, n in
                let host = Unmanaged<LoopbackHost>.fromOpaque(ctx!).takeUnretainedValue()
                let count = Int(n)
                if let data, count > 0 {
                    host.data.append(Data(bytes: data, count: count))
                }
            },
            log: nil,
            tx_report: nil,
            rx_report: nil
        )
    }
}

final class LoopbackTests: XCTestCase {
    func testVersionAndDefaults() {
        let ver = String(cString: vfm_version())
        XCTAssertTrue(ver.hasPrefix("VardeFM "), ver)
        var cfg = VfmConfig()
        vfm_config_default(&cfg)
        XCTAssertEqual(cfg.mode, UInt8(VFM_MODE_NARROW))
        XCTAssertLessThanOrEqual(cfg.max_level, 6)
    }

    func testTwoModemSession() {
        let aHost = LoopbackHost()
        let bHost = LoopbackHost()
        var cfg = VfmConfig()
        vfm_config_default(&cfg)
        cfg.decode_threads = 1

        var aCbs = aHost.callbacks()
        var bCbs = bHost.callbacks()
        let aCtx = Unmanaged.passUnretained(aHost).toOpaque()
        let bCtx = Unmanaged.passUnretained(bHost).toOpaque()

        cfg.seed = 1
        let a = vfm_new(&cfg, &aCbs, aCtx)
        cfg.seed = 2
        let b = vfm_new(&cfg, &bCbs, bCtx)
        XCTAssertNotNil(a)
        XCTAssertNotNil(b)
        aHost.modem = a
        bHost.modem = b
        defer {
            vfm_free(a)
            vfm_free(b)
        }

        "MYCALL LA1AAA".withCString { vfm_command(a, $0) }
        "CONNECT LA1AAA LA2BBB".withCString { vfm_command(a, $0) }
        "MYCALL LA2BBB".withCString { vfm_command(b, $0) }
        "LISTEN ON".withCString { vfm_command(b, $0) }

        let payload = Data((0..<800).map { UInt8(($0 * 7) & 0xff) })
        var sent = false
        var airA = [Float](repeating: 0, count: 480)
        var airB = [Float](repeating: 0, count: 480)

        for _ in 0..<40_000 {
            if aHost.ptt {
                vfm_advance(a, 480)
            } else {
                airB.withUnsafeBufferPointer { vfm_rx_audio(a, $0.baseAddress, UInt($0.count)) }
            }
            airA.withUnsafeBufferPointer { vfm_rx_audio(b, $0.baseAddress, UInt($0.count)) }

            airA = take(from: aHost, modem: a!)
            airB = take(from: bHost, modem: b!)

            if !sent && bHost.lines.contains(where: { $0.hasPrefix("CONNECTED") }) {
                payload.withUnsafeBytes { raw in
                    vfm_send(b, raw.bindMemory(to: UInt8.self).baseAddress, UInt(payload.count))
                }
                sent = true
            }
            if aHost.data.count >= payload.count {
                XCTAssertEqual(aHost.data.prefix(payload.count), payload)
                return
            }
        }
        XCTFail("session did not deliver data; A lines=\(aHost.lines) B lines=\(bHost.lines)")
    }

    private func take(from host: LoopbackHost, modem: OpaquePointer) -> [Float] {
        var out = [Float](repeating: 0, count: 480)
        let n = min(480, host.queue.count)
        if n > 0 {
            out.replaceSubrange(0..<n, with: host.queue.prefix(n))
            host.queue.removeFirst(n)
            host.played += UInt64(n)
        }
        vfm_tx_played(modem, host.played)
        if !host.ptt {
            out = [Float](repeating: 0, count: 480)
        }
        return out
    }
}
