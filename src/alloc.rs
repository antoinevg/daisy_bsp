extern crate alloc;
use static_alloc::Bump;

#[global_allocator]
static A: Bump<[u8; 1 << 12]> = Bump::uninit(); // 4 kB

#[alloc_error_handler]
fn on_oom(_layout: core::alloc::Layout) -> ! {
    cortex_m::asm::bkpt();
    loop {}
}
