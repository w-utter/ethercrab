use ethercrab_wire::EtherCrabWireReadSized;

    /// TODO: docs
pub mod abort_code;
    /// TODO: docs
pub mod services;

pub use services::{ObjectDescriptionListQuery, ObjectDescriptionListQueryCounts};

/// Defined in ETG1000.6 Table 29 – CoE elements
#[derive(Clone, Copy, Debug, PartialEq, Eq, ethercrab_wire::EtherCrabWireReadWrite)]
#[cfg_attr(test, derive(arbitrary::Arbitrary))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
    /// TODO: docs
pub enum CoeService {
    /// Emergency
    Emergency = 0x01,
    /// SDO Request
    SdoRequest = 0x02,
    /// SDO Response
    SdoResponse = 0x03,
    /// TxPDO
    TxPdo = 0x04,
    /// RxPDO
    RxPdo = 0x05,
    /// TxPDO remote request
    TxPdoRemoteRequest = 0x06,
    /// RxPDO remote request
    RxPdoRemoteRequest = 0x07,
    /// SDO Information
    SdoInformation = 0x08,
}

/// The field near the bottom of SDO definition tables called "Command specifier".
///
/// See e.g. ETG1000.6 Section 5.6.2.6.2 Table 39 – Upload SDO Segment Response.
#[derive(Clone, Copy, Debug, PartialEq, Eq, ethercrab_wire::EtherCrabWireReadWrite)]
#[wire(bits = 3)]
#[repr(u8)]
    /// TODO: docs
pub enum CoeCommand {
    /// TODO: docs
    Download = 0x01,
    /// TODO: docs
    Upload = 0x02,
    /// TODO: docs
    Abort = 0x04,
    /// TODO: docs
    UploadSegment = 0x03,
}

/// Defined in ETG1000.6 Section 5.6.2.1.1
#[derive(Clone, Copy, Debug, PartialEq, Eq, ethercrab_wire::EtherCrabWireReadWrite)]
#[wire(bytes = 4)]
    /// TODO: docs
pub struct InitSdoHeader {
    #[wire(bits = 1)]
    /// TODO: docs
    pub size_indicator: bool,
    #[wire(bits = 1)]
    /// TODO: docs
    pub expedited_transfer: bool,
    #[wire(bits = 2)]
    /// TODO: docs
    pub size: u8,
    #[wire(bits = 1)]
    /// TODO: docs
    pub complete_access: bool,
    #[wire(bits = 3)]
    /// TODO: docs
    pub command: CoeCommand,
    #[wire(bytes = 2)]
    /// TODO: docs
    pub index: u16,
    #[wire(bytes = 1)]
    /// TODO: docs
    pub sub_index: u8,
}

/// Defined in ETG1000.6 5.6.2.3.1
#[derive(Clone, Copy, Debug, PartialEq, Eq, ethercrab_wire::EtherCrabWireReadWrite)]
#[wire(bytes = 1)]
    /// TODO: docs
pub struct SegmentSdoHeader {
    #[wire(bits = 1)]
    /// TODO: docs
    pub is_last_segment: bool,

    /// Segment data size, `0x00` to `0x07`.
    #[wire(bits = 3)]
    pub segment_data_size: u8,

    #[wire(bits = 1)]
    /// TODO: docs
    pub toggle: bool,

    #[wire(bits = 3)]
    command: CoeCommand,
}

/// Defined in ETG.1000.6 5.6.3.2
#[derive(Clone, Copy, Debug, PartialEq, Eq, ethercrab_wire::EtherCrabWireReadWrite)]
#[wire(bytes = 4)]
    /// TODO: docs
pub struct SdoInfoHeader {
    #[wire(bits = 7)]
    /// TODO: docs
    pub op_code: SdoInfoOpCode,
    #[wire(bits = 1)]
    /// TODO: docs
    pub incomplete: bool,
    #[wire(pre_skip = 8, bytes = 2)]
    /// TODO: docs
    pub fragments_left: u16,
}

/// Defined in ETG.1000.6 5.6.3.2
#[derive(Clone, Copy, Debug, PartialEq, Eq, ethercrab_wire::EtherCrabWireReadWrite)]
#[repr(u8)]
    /// TODO: docs
pub enum SdoInfoOpCode {
    /// TODO: docs
    GetObjectDescriptionListRequest = 0x01,
    /// TODO: docs
    GetObjectDescriptionListResponse = 0x02,
    /// TODO: docs
    GetObjectDescriptionRequest = 0x03,
    /// TODO: docs
    GetObjectDescriptionResponse = 0x04,
    /// TODO: docs
    GetEntryDescriptionRequest = 0x05,
    /// TODO: docs
    GetEntryDescriptionResponse = 0x06,
    /// TODO: docs
    SdoInfoErrorRequest = 0x07,
}

/// Subindex access.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SubIndex {
    /// Complete access.
    ///
    /// Accesses the entire entry as a single slice of data.
    Complete,

    /// Individual sub-index access.
    Index(u8),
}

impl SubIndex {
    pub(crate) fn complete_access(&self) -> bool {
        matches!(self, Self::Complete)
    }

    pub(crate) fn sub_index(&self) -> u8 {
        match self {
            // 0th sub-index counts number of sub-indices in object, so we'll start from 1
            SubIndex::Complete => 1,
            SubIndex::Index(idx) => *idx,
        }
    }
}

impl From<u8> for SubIndex {
    fn from(value: u8) -> Self {
        Self::Index(value)
    }
}

/// A trait for types that can be transferred with a single expedited SDO upload.
pub(crate) trait SdoExpedited: EtherCrabWireReadSized {}

impl SdoExpedited for u8 {}
impl SdoExpedited for u16 {}
impl SdoExpedited for u32 {}

#[cfg(test)]
mod tests {
    pub use super::*;
    use ethercrab_wire::{EtherCrabWireRead, EtherCrabWireWriteSized};

    #[test]
    fn sanity_coe_service() {
        assert_eq!(CoeService::SdoRequest.pack(), [0x02]);
        assert_eq!(
            CoeService::unpack_from_slice(&[0x02]),
            Ok(CoeService::SdoRequest)
        );
    }
}
