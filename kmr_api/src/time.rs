/// Read-only view of the runtime clock handed to a controller each tick.
///
/// Borrowing newtype: it holds a `&kmr_core::Time` the runtime lends for the
/// duration of the call, never an owned copy — so wrapping it at the boundary
/// is free and `kmr_core::Time` never appears in a public signature.
pub struct Time<'a>(&'a kmr_core::Time);

impl<'a> Time<'a> {
    /// Wrap a borrowed core clock. Called only at the api→core boundary
    /// (`ApiInline::call`); never by users.
    pub(crate) fn new(inner: &'a kmr_core::Time) -> Self {
        Time(inner)
    }
}
