# Information Theory
> This is a study note for the following article(s) 
> https://machinelearningmastery.com/what-is-information-entropy/


## Calculate the information of an event
Rare events are more uncertain or more surprising and require more information to represent them than common events.

We can calculate the amount of information there is in an event using the probability of the event. This is called “Shannon information,” “self-information,” or simply the “information,” and can be calculated for a discrete event x as follows:

```math
information(x) = -log_2(p(x))
```
where p(x) is the probability of a discrete event $x$.

The calculation of information is often written as $h(x)$; for example:

```math
h(x) = -log_2(p(x))
```
The negative sign ensures that the result is always positive or zero.

Information will be zero when the probability of an event is 1.0 or a certainty, e.g. there is no surprise.

**Low probability event has higher information (more surprise)**

## Calculate the information of a random variable

The information of a random variable $x$ with probability distribution $p$ can be written as
$H(x)$. This is called *entropy*.

```math
H(x) = -\sum_i p(x)log_2(p(x))
```

The lowest entropy will be calculated from a distribution that has a single state with probability of 1.0, while the highest entropy will be calculated from a distribution if all events are equally likely.