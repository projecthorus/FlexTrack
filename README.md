# FlexTrack - Project Horus Fork
Fork of daveake's FlexTrack software, as used for the Project Horus cutdown / mission control payload.

We've been using a modified version of FlexTrack for some time. This is an attempt to clean up the codebase, and add additional functionality as is required for some upcoming balloon launches.

The main differences to FlexTrack master are:
- Targeted at LoRa payloads only.
- No RTTY/APRS support.
- Only use binary packets.
- Additional uplink packets.


NOTE: This code will be in a somewhat broken state for a while, while we merge in the changes....