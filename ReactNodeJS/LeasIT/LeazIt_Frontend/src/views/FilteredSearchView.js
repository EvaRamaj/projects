import { DataSearch, ReactiveBase, ReactiveList } from '@appbaseio/reactivesearch-native'; // Version can be specified in package.json

// Importing other libraries
import Expo from 'expo';
import React, { Component } from 'react';
import {
    FlatList,
    Image,
    Platform,
    ScrollView,
    StatusBar,
    TouchableOpacity,
    View,
} from 'react-native';
import { DrawerNavigator } from 'react-navigation'; // Version can be specified in package.json
import { Body, Button, Header, Left, Spinner, Text, Title } from 'native-base'; // Version can be specified in package.json
import { FontAwesome as Icon, Ionicons } from '@expo/vector-icons'; // Version can be specified in package.json
import { web } from 'react-native-communications'; // Version can be specified in package.json


import {
    DEFAULT_COLORS as COLORS,
    commonStyles as common,
} from './helpers';

import styles from './Styles';
// Imports end

const COMPONENT_DEMO = 'DataSearch';

class DataSearchView extends Component {
    render() {
        const { isReady } = this.state;
        const { navigation, ...storyProps } = this.props;  // eslint-disable-line

        const isIOS = Platform.OS === 'ios';

        const headerColor = isIOS ? '#1A237E' : 'white';

        if (!isReady) {
            return (
                <View style={common.alignCenter}>
                    <StatusBar
                        backgroundColor={COLORS.primary}
                        barStyle="light-content"
                    />
                    {this.renderStatusBar()}
                    <Spinner color={COLORS.primary} />
                </View>
            );
        }

        const header = (
            <Header style={{ alignSelf: 'center' }}>
                <Left style={{ flex: 0 }}>
                    <Button
                        transparent
                        onPress={() => navigation.navigate('DrawerToggle')}
                    >
                        <Ionicons
                            name="md-menu"
                            size={25}
                            color={headerColor}
                        />
                    </Button>
                </Left>
                <Body style={{ flex: 1, justifyContent: 'center', alignItems: 'center' }}>
                <Title
                    style={{
                        color: headerColor,
                        fontSize: 18,
                        textAlign: 'center',
                        left: -5,
                    }}
                >
                    { COMPONENT_DEMO }
                </Title>
                </Body>
            </Header>
        );

        const componentMarkup = (
            <View style={styles.componentContainer}>
                <DataSearch
                    componentId="DataSearchSensor"
                    dataField={[
                        'original_title',
                        'original_title.search',
                        'authors',
                        'authors.search',
                    ]}
                    placeholder="Search for a book title or an author"
                    {...storyProps} // injecting props from navigator drawer story
                />
            </View>
        );

        return (
            <ReactiveBase
                app={APPBASE_CONFIG.app}
                credentials={APPBASE_CONFIG.credentials}
                type={APPBASE_CONFIG.type}
            >
                <View style={{ paddingTop: StatusBar.currentHeight, backgroundColor: COLORS.primary }}/>
                {this.renderStatusBar()}
                {header}
                {componentMarkup}
                <ScrollView>
                    <View style={[common.container, common.column]}>
                        <View
                            style={[common.fullWidth, common.alignCenter, styles.results]}
                        >
                            <ReactiveList
                                componentId="ReactiveList"
                                dataField="original_title"
                                size={5}
                                onAllData={this.onAllData}
                                pagination
                                paginationAt="bottom"
                                react={{
                                    and: ['DataSearchSensor'],
                                }}
                                showResultStats={false}
                                defaultQuery={
                                    () => ({
                                        query: {
                                            match_all: {},
                                        },
                                    })
                                }
                            />
                        </View>
                    </View>
                </ScrollView>
            </ReactiveBase>
        );
    }

    state = {
        isReady: false,
    }

    async componentWillMount() {
        await Expo.Font.loadAsync({
            Roboto: require('native-base/Fonts/Roboto.ttf'), // eslint-disable-line global-require
            Roboto_medium: require('native-base/Fonts/Roboto_medium.ttf'), // eslint-disable-line global-require
            Ionicons: require('native-base/Fonts/Ionicons.ttf'), // eslint-disable-line global-require
        });

        this.setState({
            isReady: true,
        });
    }

    itemCardMarkup = bookData => (
        <TouchableOpacity
            onPress={
                () => web(`https://google.com/search?q=${bookData.original_title}`)
            }
        >
            <View style={[styles.fullWidth, styles.booksRow]}>
                <View style={styles.booksRowContainer}>
                    <Image
                        source={{
                            uri: bookData.image_medium,
                        }}
                        style={styles.booksImage}
                    />
                </View>
                <View style={styles.bookInfoSection}>
                    <Text style={styles.bookTitle}>
                        { bookData.title }
                    </Text>
                    <Text style={styles.bookAuthorSection}>
                        <Text style={styles.bookAuthor}>
                            { bookData.authors }
                        </Text>
                    </Text>
                    <Text style={styles.bookPublication}>
                        Pub {bookData.original_publication_year}
                    </Text>
                    <View style={styles.bookStars}>
                        {
                            [...Array(bookData.average_rating_rounded)]
                                .map((e, i) => (
                                    <Icon
                                        key={i} // eslint-disable-line react/no-array-index-key
                                        name="star"
                                        size={20}
                                        color="gold"
                                    />
                                ))
                        }
                        <Text style={styles.bookRatings}>
                            ({bookData.average_rating})
                        </Text>
                    </View>
                </View>
            </View>
        </TouchableOpacity>
    )

    onAllData = items => (
        <FlatList
            style={styles.listContainer}
            data={items || []}
            keyExtractor={item => item.id}
            renderItem={
                ({ item }) => this.itemCardMarkup(item)
            }
        />
    )

    renderStatusBar = () => (
        <StatusBar
            backgroundColor={COLORS.primary}
            barStyle="light-content"
        />
    )
}

// Navigation Code

const navigationOptionsBuilder = (drawerLabel, iconName) => ({
    drawerLabel,
    drawerIcon: iconName ? ({ tintColor, focused }) => ( // eslint-disable-line react/prop-types
        <Ionicons
            name={focused ? `${iconName}` : `${iconName}-outline`}
            size={26}
            style={{ color: tintColor }}
        />
    ) : null,
});

const RootDrawer = DrawerNavigator({
    basic: {
        navigationOptions: navigationOptionsBuilder('Basic', 'ios-home'),
        screen: DataSearchView,
    },
    withIconPosition: {
        navigationOptions: navigationOptionsBuilder('With iconPosition'),
        screen: ({ navigation }) => ( // eslint-disable-line
            <DataSearchView
                showFilter={false}
                iconPosition="right"
                navigation={navigation}
            />
        ),
    },
    withoutSearchIcon: {
        navigationOptions: navigationOptionsBuilder('Without search icon'),
        screen: ({ navigation }) => ( // eslint-disable-line
            <DataSearchView
                showFilter={false}
                showIcon={false}
                navigation={navigation}
            />
        ),
    },
    withoutAutosuggest: {
        navigationOptions: navigationOptionsBuilder('Without autosuggest'),
        screen: ({ navigation }) => ( // eslint-disable-line
            <DataSearchView
                autosuggest={false}
                showFilter={false}
                navigation={navigation}
            />
        ),
    },
    withCustomStyles: {
        navigationOptions: navigationOptionsBuilder('With custom styles'),
        screen: ({ navigation }) => ( // eslint-disable-line
            <DataSearchView
                showFilter={false}
                defaultSelected="Harry Potter"
                innerStyle={{
                    icon: {
                        color: 'purple',
                    },
                    input: {
                        color: 'purple',
                    },
                    label: {
                        color: '#9900cc',
                    },
                    header: {
                        backgroundColor: 'purple',
                    },
                    checkbox: {
                        color: 'purple',
                    },
                }}
                navigation={navigation}
            />
        ),
    },
    withDefaultSuggestions: {
        navigationOptions: navigationOptionsBuilder('With defaultSuggestions'),
        screen: ({ navigation }) => ( // eslint-disable-line
            <DataSearchView
                showFilter={false}
                defaultSuggestions={[
                    { label: 'Sherlock Holmes', value: 'Sherlock Holmes' },
                    { label: 'The Lord of the Rings', value: 'The Lord of the Rings' },
                ]}
                navigation={navigation}
            />
        ),
    },
    withDefaultSelected: {
        navigationOptions: navigationOptionsBuilder('With defaultSelected'),
        screen: ({ navigation }) => ( // eslint-disable-line
            <DataSearchView
                showFilter={false}
                defaultSelected="Harry Potter"
                navigation={navigation}
            />
        ),
    },
    withFuzzinessAsAuto: {
        navigationOptions: navigationOptionsBuilder('With fuzziness'),
        screen: ({ navigation }) => ( // eslint-disable-line
            <DataSearchView
                showFilter={false}
                fuzziness="AUTO"
                navigation={navigation}
            />
        ),
    },
    playground: {
        navigationOptions: navigationOptionsBuilder('Playground', 'ios-flask'),
        screen: ({ navigation }) => ( // eslint-disable-line
            <DataSearchView
                // title="DataSearch: Books..."
                defaultSelected="Harry Potter"
                autosuggest
                fieldWeights={[1, 3]}
                fuzziness={1}
                queryFormat="or"
                showFilter
                iconPosition="left"
                filterLabel="Books filter"
                highlight={false}
                innerStyle={{
                    icon: {
                        color: 'purple',
                    },
                    input: {
                        color: 'purple',
                    },
                    label: {
                        color: '#9900cc',
                    },
                    header: {
                        backgroundColor: 'purple',
                    },
                    checkbox: {
                        color: 'purple',
                    },
                }}
                navigation={navigation}
            />
        ),
    },
});