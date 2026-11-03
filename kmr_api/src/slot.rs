//! The latest-value slot between two threads (research R15; SRS section 4 Constraints): one
//! writer, one reader, never a lock or an allocation after it is built.

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

use triple_buffer::{Input, Output, triple_buffer};

/// A latest-value slot for a `Copy` value, built once and split into its [`LatestWriter`] and its
/// [`LatestReader`]. A put never waits for the reader; a take returns each value once. A value
/// overwritten before it was taken is counted as superseded.
///
/// The writer costs one atomic swap per put. The reader costs one atomic load when nothing is
/// new, one swap when something is. Neither side allocates, locks or waits.
pub struct Latest<T: Copy + Send> {
    writer: LatestWriter<T>,
    reader: LatestReader<T>,
}

/// The writing half of a [`Latest`]: one thread puts.
pub struct LatestWriter<T: Copy + Send> {
    input: Input<Option<T>>,
    superseded: Arc<AtomicU64>,
}

/// The reading half of a [`Latest`]: one thread takes.
pub struct LatestReader<T: Copy + Send> {
    output: Output<Option<T>>,
    superseded: Arc<AtomicU64>,
}

impl<T: Copy + Send> Latest<T> {
    /// An empty slot: the first take returns `None` until a put.
    pub fn new() -> Self {
        let (input, output) = triple_buffer(&None);
        let superseded = Arc::new(AtomicU64::new(0));
        Self {
            writer: LatestWriter {
                input,
                superseded: Arc::clone(&superseded),
            },
            reader: LatestReader { output, superseded },
        }
    }

    /// The two halves, each for its own thread.
    pub fn split(self) -> (LatestWriter<T>, LatestReader<T>) {
        (self.writer, self.reader)
    }
}

impl<T: Copy + Send> Default for Latest<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Copy + Send> LatestWriter<T> {
    /// Makes `value` the latest; an untaken earlier value is dropped and counted.
    pub fn put(&mut self, value: T) {
        *self.input.input_buffer_mut() = Some(value);
        if self.input.publish() {
            self.superseded.fetch_add(1, Ordering::Relaxed);
        }
    }

    /// Values overwritten before the reader took them, since the slot was built.
    pub fn superseded(&self) -> u64 {
        self.superseded.load(Ordering::Relaxed)
    }
}

impl<T: Copy + Send> LatestReader<T> {
    /// The latest value if it was put since the last take, else `None`.
    pub fn take(&mut self) -> Option<T> {
        if self.output.update() {
            *self.output.output_buffer()
        } else {
            None
        }
    }

    /// Values overwritten before this reader took them, since the slot was built.
    pub fn superseded(&self) -> u64 {
        self.superseded.load(Ordering::Relaxed)
    }
}

#[cfg(test)]
mod tests;
