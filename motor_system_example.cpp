#include "motor_system.hpp"

// --- セットアップ ---

static MotorSystem  system;
static MotorAxis    x_axis, y_axis, z_axis;

void setup() {
    system.add_motor(&x_axis);
    system.add_motor(&y_axis);
    system.add_motor(&z_axis);

    // 初期状態: 依存なし → X, Y, Z の順はソート任せ
    system.init();
}

// --- ISR (1 ms) ---

void isr_1ms() {
    system.isr_cycle();
    // この時点で active() が指すリストは常に完結したトポロジカル順。
    // メインループが途中で依存を変えていても、このサイクルは
    // commit() 前の古い順か commit() 後の新しい順のどちらかで動く。
}

// --- メインループ ---

void main_loop() {
    // ケース1: Y を X のフォロワーにする (X → Y の依存)
    //
    //   X ──▶ Y    Z (独立)
    //
    // 期待されるディスパッチ順: X, Z, Y  (または X, Y, Z でも可)
    bool ok = system.set_dependency(&y_axis, &x_axis);
    // ok == true、active() が指すリストが切り替わっている

    // ケース2: さらに Z を Y のフォロワーにする
    //
    //   X ──▶ Y ──▶ Z
    //
    // 期待されるディスパッチ順: X, Y, Z
    ok = system.set_dependency(&z_axis, &y_axis);
    // ok == true

    // ケース3: 循環依存を試みる (X を Z のフォロワーにしようとする)
    //
    //   X ──▶ Y ──▶ Z ──▶ X  ← 循環!
    //
    ok = system.set_dependency(&x_axis, &z_axis);
    // ok == false
    // graph は変更されず、active buffer も変わらない。
    // エラー処理をここで行う。

    // ケース4: 依存を外す
    ok = system.set_dependency(&z_axis, nullptr);
    //
    //   X ──▶ Y    Z (独立に戻る)
    //
    // ok == true
}
