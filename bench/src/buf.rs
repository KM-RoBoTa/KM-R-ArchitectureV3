//! Fixed-capacity byte buffer: the bench formats text with `write!` into it, never on the heap.

use std::fmt;

/// Up to `CAP` bytes; a write that does not fit is refused whole.
pub struct Buf<const CAP: usize> {
    bytes: [u8; CAP],
    len: usize,
}

impl<const CAP: usize> Buf<CAP> {
    /// An empty buffer.
    pub const fn new() -> Self {
        Self {
            bytes: [0; CAP],
            len: 0,
        }
    }

    /// Appends `more` whole, or returns `false` and leaves the buffer as it was.
    pub fn push(&mut self, more: &[u8]) -> bool {
        let end = self.len.saturating_add(more.len());
        match self.bytes.get_mut(self.len..end) {
            Some(room) => {
                room.copy_from_slice(more);
                self.len = end;
                true
            }
            None => false,
        }
    }

    /// Empties the buffer.
    pub fn clear(&mut self) {
        self.len = 0;
    }

    /// The bytes written so far.
    pub fn as_bytes(&self) -> &[u8] {
        self.bytes.get(..self.len).unwrap_or_default()
    }

    /// The text written so far; empty if raw bytes made it invalid UTF-8.
    pub fn as_str(&self) -> &str {
        std::str::from_utf8(self.as_bytes()).unwrap_or_default()
    }

    /// Number of bytes written.
    pub fn len(&self) -> usize {
        self.len
    }

    /// Whether nothing was written.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

impl<const CAP: usize> Default for Buf<CAP> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const CAP: usize> fmt::Write for Buf<CAP> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        if self.push(s.as_bytes()) {
            Ok(())
        } else {
            Err(fmt::Error)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fmt::Write;

    #[test]
    fn a_write_that_does_not_fit_is_refused_whole() {
        let mut b = Buf::<8>::new();
        assert!(write!(b, "dt={}", 1000).is_ok());
        assert!(write!(b, "xyz").is_err());
        assert_eq!(b.as_str(), "dt=1000");
        assert!(b.push(b"!"));
        assert_eq!((b.as_str(), b.len()), ("dt=1000!", 8));
    }
}
