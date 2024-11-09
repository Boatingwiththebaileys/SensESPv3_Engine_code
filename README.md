# SensESPv3_Engine_code
Engine Code example running on SensESPv3 - UNTESTED

This is the same code that I was running on my ESP on V2, it's just migrated to the new v3 framework. They have made several improvements, especially around the WiFi manager. 

This will build, but it's not tested at present. Also, I've made a couple of changes, the 1-wire sensors are all on the same pin in this version and I've removed the raw output for the Bilge monitor so it will just say Water present or Bilge Clear and not sending 1/0 at the same time. The code won't compile yet with that line in and I need to figure out why.

Fair Winds!
The Baileys
