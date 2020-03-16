"use strict";

import React from 'react';
import { Link } from 'react-router-dom'
import { Card, CardTitle, CardText, Media, MediaOverlay, Grid, Cell, Button, FontIcon } from 'react-md';

import Page from './Page';

import AuthService from '../services/AuthService';

export class ItemEvaluationDetail extends React.Component {

    constructor(props) {
        super(props);
    }

    render() {
        return (
            <Page>
                <Card>
                    <Grid className="grid-example" >
                        <Cell size={3}>
                            <Media aspectRatio="1-1">

                            </Media>
                        </Cell>
                        <Cell size={7}/>
                        <Cell size={1}>
                            {AuthService.isAuthenticated() ?
                                <Link to={{pathname: `/edit/${this.props.movie._id}`, state : {movie : this.props.movie}}}><Button icon>mode_edit</Button></Link>
                                : <Link to={'/login'}><Button icon>mode_edit</Button></Link>
                            }
                        </Cell>
                        <Cell size={1}>
                            {AuthService.isAuthenticated() ?
                                <Button onClick={() => this.props.onDelete(this.props.movie._id)} icon>delete</Button>
                                :   <Link to={'/login'}><Button icon>delete</Button></Link>
                            }
                        </Cell>
                    </Grid>

                    <CardTitle title={this.props.movie.title} subtitle={this.props.movie.year} />

                    <CardText>
                        <p>
                            {this.props.movie.mpaa_rating}
                        </p>
                        <p>
                            {this.props.movie.synopsis}
                        </p>
                    </CardText>
                </Card>
            </Page>
        );
    }
}