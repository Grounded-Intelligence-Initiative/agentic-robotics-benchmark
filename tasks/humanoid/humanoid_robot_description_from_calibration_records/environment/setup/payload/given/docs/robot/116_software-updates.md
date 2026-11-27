# Software Updates

> Update the whole robot with one signed file from the Software Update page.

- Section: Humanoid › Operate the humanoid robot

The humanoid robot updates as one unit: a single signed file updates the robot's apps and the Radxa CM5 firmware for the Robot Processing Unit (RPU) together. You install it from the **Software Update** page in Humanoid Manager, with no terminal needed.



## Get the Update File

Updates come as a signed `.mender` file of up to 2 GB. Under **Get updates**, **Check Now** asks the maker's signed feed whether a newer update exists and **Download** fetches it; or add a file by hand under **Add an update file**.

## Install an Update

1. Support the robot and turn it off. The update restarts the RPU.
2. Open **Software Update** and get the update: under **Get updates**, select **Check Now** and then **Download**, or select **Choose File…** and pick the `.mender` file.
3. Select **Update Now**, then **Install** to confirm. You can close the page — the update continues on the robot, and the page shows its progress when you come back.

An update isn't finished when it installs: the robot runs a health check on its services first, then reports the result.

| Result | Meaning |
| - | - |
| **Updated to Robot Software \<version>** | Done. The services stayed healthy for the whole check |
| **Update rolled back** | Something failed the health check, and the robot returned to the previous version |
| **Update undone** or **Update ended** | The update was reversed or abandoned before it finished |
| **Update failed** | It didn't install. The log on the page says why |

Select **OK** to dismiss the result. If the page instead says an update was interrupted — for example **The robot restarted during an update** — select **Finish Update** to run the health check again.

> [!NOTE]
>
> **Some messages are for a technician**
>
> Messages such as **The RPU kept the new version** point to recovery steps that are run over SSH. If you see one without having done anything unusual, note the exact message and contact support rather than improvising.

## Check Versions

- **Software Update → Installed** lists **Robot software** and **RPU** — the CM5 firmware.
- **Settings** shows the console's own version under **Software**, which matches the release.

Updates don't change actuator IDs, calibration or the assembly confirmation.
