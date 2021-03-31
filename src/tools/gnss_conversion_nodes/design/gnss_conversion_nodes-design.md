GNSS Conversion Node {#gnss-conversion-nodes-design}
=========================

# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

In order to work with GNSS output in localization module there is a need for a node that would transform the GNSS input into a metric pose. This node does exactly that.


# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->
The node subscribes to the GNSS messages and publishes `autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped` converted messages setting the covariance from the user-provided values.

## Inner-workings / Algorithms
<!-- If applicable -->

This node implements a conversion from WGS 84 (latitude, longitude, height) measurement into an ECEF (earth-centered, earth-fixed) metric pose \f$[x, y, z]^\top\f$ following the following equations (given `[latitude, longitude, height]` = \f$[\phi,\lambda,h]^\top\f$):

\f{aligned}{
x =& (N(\phi) + h) \cos\phi \cos\lambda\\
y =& (N(\phi) + h) \cos\phi \sin\lambda\\
z =& ((1 - e^2) N(\phi) + h) \sin\phi,
\f}

where \f$N(\phi) = \frac{a}{\sqrt{1 - e^2 \sin^2 \phi}}\f$. Here, \f$a = 6378137\f$, and \f$e^2 = 6.69437999014 \times 10^{−3}\f$.


## Assumptions / Known limits
<!-- Required -->
- For now only a transformation from WGS84 to ECEF is supported.
- For now only the user-provided covariance is used, the one set in the original message is not propagated

# Related issues
- #768 - GNSS Localization
