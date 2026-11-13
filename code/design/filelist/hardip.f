// ===================================================================
// hardip.f — 硬核 IP（SRAM）filelist
//
// 版本：v2.0 | 日期：2026-10-26 | 作者：OpenHappy Team
// 修订：v2.0 去厂商依赖改造 —— 删除全部厂商提供的 SRAM 仿真模型
//       （<macro>_core.v / <macro>_ikos.v），改用手写 RTL 行为
//       模型 sram_1rw_128b.v（参数化 DEPTH/ADDR_W）。
//
// -------------------------------------------------------------------
// 说明
// -------------------------------------------------------------------
//   HIPINEXT_HOME points at the repository root's code/ directory (symm_d2d layout removed);
//   hardip/ is its subdirectory.
//
//   当前包含：
//     hardip/sram/sram_1rw_128b.v
//       —— 128 bit 宽 1RW SRAM 行为模型，参数化深度（32/64/128）。
//          供 link_layer/replay_buffer.v 例化（4 个 ifdef 分支按 DEPTH 选择）。
//
//   已移除（原 v1.x 内容）：
//     hardip/sram/ 下的厂商 SRAM 仿真模型（<macro>_core.v / <macro>_ikos.v）
//       —— 已被 sram_1rw_128b.v 替代（replay_buffer 是唯一使用者）。
//     厂商 MCU 子系统及其 SRAM 封装模型
//       —— 不在开源范围，相关 RTL（mcu_subsys_new.v / mcu_subsys_sram_sp_8192x32.v）
//          与其 filelist（mcu_subsys_new.f / third_ip.f）已一并移除。
//          注：logic_phy/mcu_subsys.v 为端口 stub（MCU_SUBSYS 宏默认关闭），
//          保留不作为本 filelist 内容。
//
// -------------------------------------------------------------------
// 综合／后端替换说明
// -------------------------------------------------------------------
//   本 filelist 仅用于 RTL 功能仿真。综合与后端流程需按 sram_1rw_128b 的
//   端口契约（clk/req/we/addr/wdata/rdata，1 拍读延迟）替换为真实 SRAM 宏，
//   替换清单另行提供。
// ===================================================================

// --- 手写 RTL SRAM 行为模型（无 vendor 依赖）---
$HIPINEXT_HOME/hardip/sram/sram_1rw_128b.v
