/// Read-only view of the robot sensors handed to a controller each tick.
///
/// Borrowing newtype over `&kmr_core::Sensors`, wrapped at the api→core
/// boundary so the core type never appears in a public signature.
pub struct Sensors<'a> {
    /// The core sensor handle the runtime lends for one tick.
    #[expect(dead_code, reason = "the core's Sensors has no reading yet to forward")]
    inner: &'a kmr_core::Sensors,
}

impl<'a> Sensors<'a> {
    /// Wrap a borrowed core sensor handle. Called only at the boundary
    /// (`ApiInline::call`); never by users.
    pub(crate) fn new(inner: &'a kmr_core::Sensors) -> Self {
        Sensors { inner }
    }
}
