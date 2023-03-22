// #include "gtest/gtest.h"
// #include "loss.h"
// using smartlabel::NodePoint;

// TEST(NodePoint_test, standard_test) {
//   NodePoint default_node;
//   EXPECT_EQ(default_node.cluster_id, -1);
//   EXPECT_EQ(default_node.prev_id, -1);
//   EXPECT_EQ(default_node.next_id, -1);

//   // raw node
//   NodePoint node(1);
//   EXPECT_EQ(node.cluster_id, 1);
//   EXPECT_EQ(node.prev_id, -1);
//   EXPECT_EQ(node.next_id, -1);

//   // should be able to set self as friend
//   EXPECT_FALSE(node.set_friend(1));
//   EXPECT_EQ(node.prev_id, -1);
//   EXPECT_EQ(node.next_id, -1);

//   // should be able to set friend
//   EXPECT_TRUE(node.set_friend(2));
//   EXPECT_EQ(node.prev_id, 2);
//   EXPECT_EQ(node.next_id, -1);

//   // should be able to check known friend
//   EXPECT_TRUE(node.set_friend(2));
//   EXPECT_EQ(node.prev_id, 2);
//   EXPECT_EQ(node.next_id, -1);

//   // should be able to set self as friend
//   EXPECT_FALSE(node.set_friend(1));
//   EXPECT_EQ(node.prev_id, 2);
//   EXPECT_EQ(node.next_id, -1);

//   // should be able to set another friend
//   EXPECT_TRUE(node.set_friend(3));
//   EXPECT_EQ(node.prev_id, 2);
//   EXPECT_EQ(node.next_id, 3);

//   EXPECT_TRUE(node.set_friend(2));
//   EXPECT_EQ(node.prev_id, 2);
//   EXPECT_EQ(node.next_id, 3);

//   EXPECT_TRUE(node.set_friend(3));
//   EXPECT_EQ(node.prev_id, 2);
//   EXPECT_EQ(node.next_id, 3);

//   EXPECT_FALSE(node.set_friend(1));
//   EXPECT_EQ(node.prev_id, 2);
//   EXPECT_EQ(node.next_id, 3);

//   EXPECT_FALSE(node.set_friend(4));
//   EXPECT_EQ(node.prev_id, 2);
//   EXPECT_EQ(node.next_id, 3);

//   EXPECT_FALSE(node.set_friend(1));
//   EXPECT_EQ(node.prev_id, 2);
//   EXPECT_EQ(node.next_id, 3);

//   EXPECT_TRUE(node.set_friend(2));
//   EXPECT_EQ(node.prev_id, 2);
//   EXPECT_EQ(node.next_id, 3);

//   EXPECT_TRUE(node.set_friend(3));
//   EXPECT_EQ(node.prev_id, 2);
//   EXPECT_EQ(node.next_id, 3);

//   // check next
//   EXPECT_EQ(node.next(-1), -1);
//   EXPECT_EQ(node.next(0), -1);
//   EXPECT_EQ(node.next(1), -1);
//   EXPECT_EQ(node.next(2), 3);
//   EXPECT_EQ(node.next(3), 2);
//   EXPECT_EQ(node.next(4), -1);
// }