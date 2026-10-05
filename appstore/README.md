# Publishing Bcad on the Mac App Store

Everything here is ready except what needs your Apple account. Follow the steps in order; each takes minutes, except
Apple's own checks (account approval: up to 2 days; App Review: usually 1–3 days).

## What's in this package
- **`Bcad.app`**: the App Store version as Apple will run it: sandboxed, hardened runtime, plans through the App Store.
  Signed only for your Mac, so its plans show "Prices couldn't be loaded" until it's signed with your account.
- **`Bcad source/`**: all of Bcad's code, with **`appstore/Bcad.xcodeproj`**, the Xcode project you upload from.

To open an app downloaded from the internet that isn't from the App Store: right-click it → **Open** → **Open**
(once). Or in Terminal: `xattr -dr com.apple.quarantine /path/to/Bcad.app`.

## 1. Apple Developer Program
- Join at <https://developer.apple.com/programs/> ($99 a year).

## 2. Register the app's ID
- <https://developer.apple.com/account/resources/identifiers> → **+** → **App IDs** → **App**.
- Description: `Bcad`. Bundle ID (Explicit): **`com.bohdan.bcad`**. In-App Purchase is on by default. Register.

## 3. Create the app in App Store Connect
- <https://appstoreconnect.apple.com> → **Apps** → **+** → **New App**.
- Platform **macOS**; name **Bcad** (if taken, e.g. "Bcad 3D"); language English; bundle ID `com.bohdan.bcad`; SKU
  `bcad`.
- **Business** (in App Store Connect): accept the **Paid Apps Agreement**, add bank and tax details. Subscriptions
  can't be sold before this.

## 4. Subscriptions (the plans)
In the app → **Monetization → Subscriptions** → create a **subscription group** named `Bcad`, then 4 subscriptions.
The **Product IDs must be exactly these** (the app looks for them):

| Reference name | Product ID | Level | Duration | Price | Free trial |
|---|---|---|---|---|---|
| Studio monthly | `bcad.studio.monthly` | 1 (highest) | 1 month | $15.00 | 1 week, new subscribers |
| Studio yearly | `bcad.studio.yearly` | 1 | 1 year | $150.00 | 1 week |
| Pro monthly | `bcad.pro.monthly` | 2 | 1 month | $9.00 | 1 week |
| Pro yearly | `bcad.pro.yearly` | 2 | 1 year | $90.00 | 1 week |

- **Level**: drag Studio above Pro in the group's list (upgrades and downgrades follow from it).
- **Free trial**: each subscription → **Subscription Prices** → **Introductory Offers** → Free, 1 week, new subscribers.
- **Localization** (each): display name "Bcad Pro" / "Bcad Studio"; description "Unlimited saving and exporting" /
  "Unlimited files and human figures". Group localization: "Bcad".
- **Review information** (each): a screenshot of the Plans card (Bcad menu → Plans…).

## 5. Privacy and terms
- **App Privacy** → **Data Collection**: **"No, we do not collect data from this app."**
- **Privacy Policy URL**: put `docs/privacy.md` online (e.g. GitHub Pages, or any web page) and paste its address.
  If it isn't `https://github.com/4dbbfjv49t-svg/Bcad/blob/main/docs/privacy.md`, change `Catalog.privacy` in
  `Plans.swift` to the same address.
- **Terms of Use**: the app links to Apple's standard licence (EULA); nothing to add. In the app's description add a
  line: "Terms of Use: https://www.apple.com/legal/internet-services/itunes/dev/stdeula/".

## 6. Build and upload
1. Install **Xcode 26** from the App Store and sign in: Xcode → Settings → Accounts → **+** → your Apple ID.
2. Open **`Bcad source/appstore/Bcad.xcodeproj`**.
3. Select the **Bcad** target → **Signing & Capabilities** → Team: **your team**. (Signing is automatic.)
4. Optional: **Product → Run** to try it. Plans can be bought there without money: the project uses `Bcad.storekit`, a
   local test store.
5. **Product → Archive**. When the Organizer opens: **Validate App**, then **Distribute App → App Store Connect →
   Upload**.
6. Next upload: raise **Build** (General tab) by 1 each time; raise **Version** for a new release.

(If you change the code: the project was made by `appstore/prepare.sh`; run it again on a Mac with
`brew install xcodegen`.)

## 7. Submit for review
In App Store Connect → the app → the version:
- Description, keywords, support URL (can be the GitHub page), screenshots (at least one, 1280×800 or larger:
  take them in Bcad with ⇧⌘5).
- **Build**: choose the uploaded build. **In-App Purchases and Subscriptions**: add the 4 subscriptions to this version.
- **App Review notes** (paste):

  > Bcad is a CAD app for 3D printing. Free: one document a day can be saved or exported (as often as needed that
  > day), human figures need Studio. Pro ($9/month, $90/year): unlimited saving and exporting. Studio ($15/month,
  > $150/year): Pro plus human figures (the person button at the end of the bottom shape bar). Each has a one-week
  > free trial. Plans: Bcad menu → Plans…, or Settings → Plan. To test: add a box, save it (File → Save), then make a
  > new document and save again to see the free limit; subscribe with a sandbox account to remove it.

- **Submit for Review**.

## What changes for people who use it
- Their files are theirs: the app reaches only files they open, save or drop on it (the sandbox), and remembers recent
  files with bookmarks.
- Renaming a saved document from the side panel shows the Save panel (the sandbox lets the app write only where it's
  told); the file under the old name stays.
- Settings and unsaved work live in the app's own container (`~/Library/Containers/com.bohdan.bcad`).
